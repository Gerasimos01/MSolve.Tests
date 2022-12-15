using System;
using System.Collections.Generic;
using System.Diagnostics;

using MGroup.Constitutive.Structural;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.Constitutive.Structural.Continuum;
using MGroup.Constitutive.Structural.Transient;
using MGroup.FEM.Structural.Continuum;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Discretization.Entities;
using MGroup.Multiscale.SupportiveClasses;


//using ContinuumElement3DFactory = MGroup.FEM.Structural.Continuum.ContinuumElement3DFactory;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{

    public class GmshCompositeRveBuilder 
    {

        double boundarySearchTol;
        IIsotropicContinuumMaterial3D matrixMaterial;
        
        string gmshRveFilePath;

        double[,] rveNodeData;
        List<int[,]> celltype2ELementsAndNodes;
        List<int[,]> celltype5ELementsAndNodes;
        List<int[,]> celltype4ELementsAndNodes;
        int[] boundaryNodesIds1;
        int[] structureNodesIds;


        public double youngModulus { get; set; } = 1353000;
        public double poissonRatio { get; set; } = 0.3;

        public double density { get; set; } = 1;
        public double load_value { get; set; } = 1;

        public StructuralDof  loadedDof { get; set; } = StructuralDof.TranslationX;
        public int monitoredNodeAId { get; set; } = 5;



        //TODO: input material to be cloned.  
        public GmshCompositeRveBuilder(string gmshRveFilePath, double boundarySearchTol = 1e-07)
        {
            this.matrixMaterial = matrixMaterial;
            this.boundarySearchTol = boundarySearchTol;
            this.gmshRveFilePath = gmshRveFilePath;

            (this.rveNodeData, this.celltype2ELementsAndNodes, this.celltype4ELementsAndNodes, this.celltype5ELementsAndNodes) = 
                GmshMultiplePhaseReader.ReadFile(gmshRveFilePath, false);

            ( boundaryNodesIds1, structureNodesIds) = SearchBoundaryNodesIds1(rveNodeData, boundarySearchTol);

            monitoredNodeAId = structureNodesIds[50];


        }

        


        public Model GetModel()
        {
            Model model = new Model();
            
            model.SubdomainsDictionary[0] = new Subdomain(0);
       
            
            // define nodes
            for (int i1 = 0; i1 < rveNodeData.GetLength(0); i1++)
            {
                int nodeID = (int)rveNodeData[i1,0];
                double nodeCoordX = rveNodeData[i1, 1];
                double nodeCoordY = rveNodeData[i1, 2];
                double nodeCoordZ = rveNodeData[i1, 3];

                model.NodesDictionary.Add(nodeID, new Node(id: nodeID, x: nodeCoordX, y: nodeCoordY, z: nodeCoordZ));
            }



            var elementCreator = new ContinuumElement3DFactory(
                new ElasticMaterial3D(youngModulus: youngModulus, poissonRatio: poissonRatio),
                commonDynamicProperties: new TransientAnalysisProperties(density, 0, 0));
            //define celltype4 elements group
            int nElementGroupsI = celltype4ELementsAndNodes.Count;
            int[] ContinuumTet4NodesNumbering = new int[4] { 0, 1, 2, 3 };
            int subdomainID = 0;
            for (int i2 = 0; i2 < nElementGroupsI ; i2++)
            {


                for (int i1 = 0; i1 < celltype4ELementsAndNodes[i2].GetLength(0); i1++)
                {
                    List<Node> nodeSet = new List<Node>();
                    for (int j = 0; j < 4; j++)
                    {
                        int ren1 = ContinuumTet4NodesNumbering[j];
                        int nodeID = celltype4ELementsAndNodes[i2][i1, ren1 + 1];
                        nodeSet.Add((Node)model.NodesDictionary[nodeID]);
                    }

					IElementType e1 = elementCreator.CreateNonLinearElement(
                    CellType.Tet4, nodeSet,
                    commonMaterial: new ElasticMaterial3D(youngModulus: youngModulus, poissonRatio: poissonRatio),
                    commonDynamicProperties: new TransientAnalysisProperties(density, 0, 0));
                    e1.ID = celltype4ELementsAndNodes[i2][i1, 0];

                    model.ElementsDictionary.Add(e1.ID, e1);
                    model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
                }

            }

            //define celltype5 elements group
            nElementGroupsI = celltype5ELementsAndNodes.Count;
            int[] Continuumhexa8NodesNumbering = new int[8] { 0, 1, 2, 3,4,5,6,7 };
            for (int i2 = 0; i2 < nElementGroupsI; i2++)
            {


                for (int i1 = 0; i1 < celltype5ELementsAndNodes[i2].GetLength(0); i1++)
                {
                    List<Node> nodeSet = new List<Node>();
                    for (int j = 0; j < 8; j++)
                    {
                        int ren1 = Continuumhexa8NodesNumbering[j];
                        int nodeID = celltype5ELementsAndNodes[i2][i1, ren1 + 1];
                        nodeSet.Add((Node)model.NodesDictionary[nodeID]);
                    }

                    IElementType e1 = elementCreator.CreateNonLinearElement(
                    CellType.Hexa8, nodeSet,
                    commonMaterial: new ElasticMaterial3D(youngModulus: youngModulus, poissonRatio: poissonRatio),
                    commonDynamicProperties: new TransientAnalysisProperties(density, 0, 0));
                    e1.ID = celltype5ELementsAndNodes[i2][i1, 0];

                    model.ElementsDictionary.Add(e1.ID, e1);
                    model.SubdomainsDictionary[subdomainID].Elements.Add(e1);
                }

            }


            // define constraints

            var constraints = new List<INodalDisplacementBoundaryCondition>();
            for (var i = 1; i < boundaryNodesIds1.Length; i++)
            {
                constraints.Add(new NodalDisplacement(model.NodesDictionary[boundaryNodesIds1[i]], StructuralDof.TranslationX, amount: 0d));
                constraints.Add(new NodalDisplacement(model.NodesDictionary[boundaryNodesIds1[i]], StructuralDof.TranslationY, amount: 0d));
                constraints.Add(new NodalDisplacement(model.NodesDictionary[boundaryNodesIds1[i]], StructuralDof.TranslationZ, amount: 0d));
            }

            var emptyloads1 = new List<INodalLoadBoundaryCondition>();

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(constraints, emptyloads1));


            return   model; 
        }


        public void AddPeriodicTransientLoadTimeFunc(Model model, Func<double, double, double> timeFunc)
        {
            var loads = new List<INodalLoadBoundaryCondition>();
            var emptyConstraints1 = new List<INodalDisplacementBoundaryCondition>();

            for (var i = 0; i < structureNodesIds.Length; i++)
            {
                loads.Add(new NodalLoad(model.NodesDictionary[structureNodesIds[i]], loadedDof, amount: load_value));
            }

            var boundaryLoadConditionSet = new StructuralBoundaryConditionSet(emptyConstraints1, loads);
            var transientLoadandBCs = new StructuralTransientBoundaryConditionSet(
                new List<IBoundaryConditionSet<IStructuralDofType>>() { boundaryLoadConditionSet },
                timeFunc);

            model.BoundaryConditions.Add(transientLoadandBCs);
        }

        public void AddStaticNodalLoads(Model model)
        {
            var emptyConstraintsconstraints = new List<INodalDisplacementBoundaryCondition>();
            var loads = new List<INodalLoadBoundaryCondition>();
            for (var i = 0; i < structureNodesIds.Length; i++)
            {
                loads.Add(new NodalLoad(model.NodesDictionary[structureNodesIds[i]], loadedDof, amount: load_value));
            }

            model.BoundaryConditions.Add(new StructuralBoundaryConditionSet(emptyConstraintsconstraints, loads));
        }




        private (int[], int[]) SearchBoundaryNodesIds1(double[,] rveNodeData,  double boundarySearchTol)
        {
            List<int> boundaryNodesIds = new List<int>();
            List<int> mainStructureNodeIds = new List<int>();

            for (int i1 = 0; i1 < rveNodeData.GetLength(0); i1++)
            {
                int nodeId = (int)rveNodeData[i1, 0];
                double X = rveNodeData[i1, 1];
                

                bool isBoundaryNode1 = false;

                if (X< boundarySearchTol)
                {                    
                    isBoundaryNode1 = true;
                }

               

                if (isBoundaryNode1)
                { boundaryNodesIds.Add(nodeId); }
                else 
                { mainStructureNodeIds.Add(nodeId); }
                               

            }



            return (boundaryNodesIds.ToArray(),mainStructureNodeIds.ToArray());
        }




    }
}
