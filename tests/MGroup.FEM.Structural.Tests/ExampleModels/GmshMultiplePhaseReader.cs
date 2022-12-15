using System;
using System.Collections.Generic;
using System.Globalization;
using System.IO;
using System.Net.NetworkInformation;
using System.Threading;

using MGroup.LinearAlgebra.Output;
//using ISAAR.MSolve.Discretization.Interfaces;

namespace MGroup.Multiscale.SupportiveClasses
{
    /// <summary>
    /// Creates meshes by reading GMSH output files (.msh). Unrecognized GMSH cell types will be ignored along with any 1D cells
    /// in the .msh file, therefore some care is needed.
    /// Authors: Serafeim Bakalakos
    /// </summary>
    public static class GmshMultiplePhaseReader 
    {
        public static (double[,], List<int[,]>, List<int[,]>, List<int[,]>) ReadFile(string path, bool printOutput)
        {
			//string filePath = $@"C:\Users\acivi\Documents\notes_elegxoi_2\developGmsh_example\entityless\t16Solid.msh";
			//string filePath = $@"C:\Users\acivi\Documents\notes_elegxoi_2\developGmsh_example\2nd iteration\t16Solid_physical_entities.msh";//.//.
			//string filePath = $@"C:\Users\acivi\Documents\notes_elegxoi_2\developGmsh_example\2nd iteration\t16Solid_physical_entities_no_volume_tag_change_More_inclusions.msh";//.//.
			string CultureName = Thread.CurrentThread.CurrentCulture.Name;
			CultureInfo ci = new CultureInfo(CultureName);
			ci.NumberFormat.NumberDecimalSeparator = ".";
			Thread.CurrentThread.CurrentCulture = ci;
			var reader = new StreamReader(path);

            string line;
            #region Find nodes segment
            while (true)
            {
                line = reader.ReadLine();
                if (line[0] == '$')
                    if (line.Equals("$Nodes")) break; // Next line will be the nodes count.
            }

            //Read the Line: numEntityBlocks(size_t) numNodes(size_t) minNodeTag(size_t) maxNodeTag(size_t)
            var firstLine = reader.ReadLine();
            string[] subStrings = firstLine.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
            int numEntityBlocks = Int32.Parse(subStrings[0]);
            int numNodes = Int32.Parse(subStrings[1]);
            int minNodeTag = Int32.Parse(subStrings[2]);
            int maxNodeTag = Int32.Parse(subStrings[3]);

            int[][] entitiesNodeIds = new int[numEntityBlocks][];
            double[][,] entitiesNodeCoords = new double[numEntityBlocks][,];
            int[] entitiesIds = new int[numEntityBlocks];
            int[] entitiesDimension = new int[numEntityBlocks];

            int numEntitities = 0;
            while (true)
            {
                line = reader.ReadLine();
                if (line == "$EndNodes")
                { break; } // Next line will be the nodes count.
                else
                {
                    numEntitities++;
                    subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    //Read the Line: entityDim(int) entityTag(int) parametric(int; 0 or 1) numNodesInBlock(size_t)
                    int entityDim = Int32.Parse(subStrings[0]); //0:Point 1:Line 2:Surface 3:Volume
                    int numTag = Int32.Parse(subStrings[1]); //p.x tag = 9
                    int isParamteric = Int32.Parse(subStrings[2]); //isParametric boolean px 0: false
                    int numNodesInBlock = Int32.Parse(subStrings[3]); //num Nodes to Follow
                    entitiesDimension[numEntitities - 1] = entityDim;

                    int[] entityNodeIds = new int[numNodesInBlock];
                    for (int i1 = 0; i1 < numNodesInBlock; i1++)
                    {
                        line = reader.ReadLine();
                        entityNodeIds[i1] = Int32.Parse(line);
                    }
                    entitiesNodeIds[numEntitities - 1] = entityNodeIds;

                    double[,] entityNodeCoords = new double[numNodesInBlock, 3];
                    for (int i1 = 0; i1 < numNodesInBlock; i1++)
                    {
                        line = reader.ReadLine();
                        subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                        entityNodeCoords[i1, 0] = Double.Parse(subStrings[0]);
                        entityNodeCoords[i1, 1] = Double.Parse(subStrings[1]);
                        entityNodeCoords[i1, 2] = Double.Parse(subStrings[2]);
                    }
                    entitiesNodeCoords[numEntitities - 1] = entityNodeCoords;
                    entitiesIds[numEntitities - 1] = numTag;

                }

            }
            #endregion


            #region Find elements segment
            while (true)
            {
                line = reader.ReadLine();
                if (line[0] == '$')
                    if (line.Equals("$Elements")) break; // Next line will be the nodes count.
            }
            var firstElementInfoLine = reader.ReadLine();
            subStrings = firstElementInfoLine.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);

            int numElemEntityBlocks = Int32.Parse(subStrings[0]);
            int numElements = Int32.Parse(subStrings[1]);

            int minElementTag = Int32.Parse(subStrings[2]);
            int maxElementTag = Int32.Parse(subStrings[3]);

            int[][] elementIdsOfEachBlock1 = new int[numElemEntityBlocks][];
            int[] elementBlocksTags = new int[numElemEntityBlocks];
            int[][,] elementNodesIdsOfEachBlock1 = new int[numElemEntityBlocks][,];
            int[] elementBlocksCellTypes = new int[numElemEntityBlocks];


            int numElementBlocks1 = 0;
            while (true)
            {
                line = reader.ReadLine();
                if (line == "$EndElements")
                { break; } // Next line will be the nodes count.
                else
                {
                    numElementBlocks1++;
                    subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    //Read the Line: entityDim(int) entityTag(int) parametric(int; 0 or 1) numNodesInBlock(size_t)
                    int entityDim = Int32.Parse(subStrings[0]); //0:Point 1:Line 2:Surface 3:Volume
                    int entityTag = Int32.Parse(subStrings[1]); //p.x tag = 9
                    elementBlocksTags[numElementBlocks1 - 1] = entityTag;
                    int elementType = Int32.Parse(subStrings[2]); // einai o arithmos gia to celltype pou xrhsimopoieitai.
                    elementBlocksCellTypes[numElementBlocks1 - 1] = elementType;
                    int numElementsInBlock = Int32.Parse(subStrings[3]); //num Nodes to Follow


                    int[] blockElementIds = new int[numElementsInBlock];
                    line = reader.ReadLine();
                    subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    int[,] entityElementNodesIds = new int[numElementsInBlock, subStrings.Length - 1];
                    blockElementIds[0] = Int32.Parse(subStrings[0]);//.
                    for (int i1 = 0; i1 < subStrings.Length - 1; i1++)
                    {
                        entityElementNodesIds[0, i1] = Int32.Parse(subStrings[i1 + 1]);
                    }

                    for (int i2 = 1; i2 < numElementsInBlock; i2++)
                    {
                        line = reader.ReadLine();
                        subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                        blockElementIds[i2] = Int32.Parse(subStrings[0]);//.
                        for (int i1 = 0; i1 < subStrings.Length - 1; i1++)
                        {
                            entityElementNodesIds[i2, i1] = Int32.Parse(subStrings[i1 + 1]);
                        }
                    }
                    elementIdsOfEachBlock1[numElementBlocks1 - 1] = blockElementIds;
                    elementNodesIdsOfEachBlock1[numElementBlocks1 - 1] = entityElementNodesIds;

                }

            }
            #endregion


            #region print elements for check
            int elementBlockToCheck1 = 6;
            int elementBlockToCheck2 = 7;
            int elementBlockToCheck3 = 8;//.

            int totalNodesNumberModel = 0;
            for (int i1 = 0; i1 < numEntityBlocks; i1++)
            {
                totalNodesNumberModel += entitiesNodeIds[i1].GetLength(0);
            }

            double[,] rveNodeData = new double[totalNodesNumberModel, 4];//.

            int positionCounter = 0;

            for (int i2 = 0; i2 < numEntityBlocks; i2++)
            {
                for (int i3 = 0; i3 < entitiesNodeIds[i2].GetLength(0); i3++)
                {
                    rveNodeData[positionCounter, 0] = entitiesNodeIds[i2][i3];
                    rveNodeData[positionCounter, 1] = entitiesNodeCoords[i2][i3, 0];
                    rveNodeData[positionCounter, 2] = entitiesNodeCoords[i2][i3, 1];
                    rveNodeData[positionCounter, 3] = entitiesNodeCoords[i2][i3, 2];
                    positionCounter++;
                }
            }
            //(new ISAAR.MSolve.LinearAlgebra.Output.Array2DWriter()).WriteToFile(rveNodeData, CnstValuesNLShell.examplePath + @"nodeCoordsAndLoads.txt");
            if (printOutput)
            {
                (new Array2DWriter()).WriteToFile(rveNodeData, @"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\rveNodeData.txt");
            }

            int surfaceElementBlockCounter = 0;
            int solidEleemntBlockCounter = 0;

            List<int[,]> celltype2ELementsAndNodes = new List<int[,]>();
            List<int[,]> celltype4ELementsAndNodes = new List<int[,]>();
            List<int[,]> celltype5ELementsAndNodes = new List<int[,]>();

            for (int i2 = 0; i2 < numElemEntityBlocks; i2++)

            {
                if (elementBlocksCellTypes[i2] == 2) //2D triangle
                {
                    int[,] ElementAndNodes = new int[elementIdsOfEachBlock1[i2].GetLength(0), elementNodesIdsOfEachBlock1[i2].GetLength(1) + 1];
                    for (int i3 = 0; i3 < elementIdsOfEachBlock1[i2].GetLength(0); i3++)
                    {
                        ElementAndNodes[i3, 0] = elementIdsOfEachBlock1[i2][i3];
                        for (int i4 = 0; i4 < elementNodesIdsOfEachBlock1[i2].GetLength(1); i4++)
                        {
                            ElementAndNodes[i3, i4 + 1] = elementNodesIdsOfEachBlock1[i2][i3, i4];
                        }

                    }
                    if (printOutput)
                    {
                        double[,] ElAndNodesDoubleArray = ConvertIntArrayToDoubleArray(ElementAndNodes);
                        (new Array2DWriter()).WriteToFile(ElAndNodesDoubleArray, $@"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\surfaceElementNodes_{surfaceElementBlockCounter}.txt");
                    }
                    celltype2ELementsAndNodes.Add(ElementAndNodes);
                    surfaceElementBlockCounter++;
                }
                else if (elementBlocksCellTypes[i2] == 4) //3D tetrahedron
                {
                    int[,] ElementAndNodes = new int[elementIdsOfEachBlock1[i2].GetLength(0), elementNodesIdsOfEachBlock1[i2].GetLength(1) + 1];
                    for (int i3 = 0; i3 < elementIdsOfEachBlock1[i2].GetLength(0); i3++)
                    {
                        ElementAndNodes[i3, 0] = elementIdsOfEachBlock1[i2][i3];
                        for (int i4 = 0; i4 < elementNodesIdsOfEachBlock1[i2].GetLength(1); i4++)
                        {
                            ElementAndNodes[i3, i4 + 1] = elementNodesIdsOfEachBlock1[i2][i3, i4];
                        }

                    }
                    if (printOutput)
                    {
                        double[,] ElAndNodesDoubleArray = ConvertIntArrayToDoubleArray(ElementAndNodes);
                        (new Array2DWriter()).WriteToFile(ElAndNodesDoubleArray, $@"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\solidElementNodes_tet_{solidEleemntBlockCounter}.txt");
                    }
                    celltype4ELementsAndNodes.Add(ElementAndNodes);
                    solidEleemntBlockCounter++;
                }
                else if (elementBlocksCellTypes[i2] == 5) //3D hexaedron
                {
                    int[,] ElementAndNodes = new int[elementIdsOfEachBlock1[i2].GetLength(0), elementNodesIdsOfEachBlock1[i2].GetLength(1) + 1];
                    for (int i3 = 0; i3 < elementIdsOfEachBlock1[i2].GetLength(0); i3++)
                    {
                        ElementAndNodes[i3, 0] = elementIdsOfEachBlock1[i2][i3];
                        for (int i4 = 0; i4 < elementNodesIdsOfEachBlock1[i2].GetLength(1); i4++)
                        {
                            ElementAndNodes[i3, i4 + 1] = elementNodesIdsOfEachBlock1[i2][i3, i4];
                        }

                    }
                    if (printOutput)
                    {
                        double[,] ElAndNodesDoubleArray = ConvertIntArrayToDoubleArray(ElementAndNodes);
                        (new Array2DWriter()).WriteToFile(ElAndNodesDoubleArray, $@"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\solidElementNodes_hexa_{solidEleemntBlockCounter}.txt");
                    }
                    celltype5ELementsAndNodes.Add(ElementAndNodes);
                    solidEleemntBlockCounter++;
                }
            }

            #endregion

            return (rveNodeData, celltype2ELementsAndNodes, celltype4ELementsAndNodes, celltype5ELementsAndNodes);
        }

        public static (int[,] TetInterfaceCohesiveNodeIds, double[,] extraNodesCoordinates) SearchCohesiveInterfacesElements(double[,] rveNodeData,
            List<int[,]> celltype2ELementsAndNodes, List<int[,]> celltype4ELementsAndNodes)
        {
            int[] ContinuumTet4NodesNumbering = new int[4] { 0, 1, 2, 3 };

            bool[,] nodeBeliingsInOutterAndInner = new bool[rveNodeData.GetLength(0), 2];

            int[,] nodeElements = new int[rveNodeData.GetLength(0), 50];
            int[,] nodeElementsGroup = new int[rveNodeData.GetLength(0), 50];
            int[,] nodeElementsPositionInTheirGroup = new int[rveNodeData.GetLength(0), 50];

            int[] columnToWriteElemen = new int[rveNodeData.GetLength(0)];

            int nElementGroupsI = celltype4ELementsAndNodes.Count;


            //outerElements 
            for (int i1 = 0; i1 < celltype4ELementsAndNodes[nElementGroupsI - 1].GetLength(0); i1++)
            {
                int elementId = celltype4ELementsAndNodes[nElementGroupsI - 1][i1, 0];
                for (int j = 0; j < 4; j++)
                {
					int ren1 = ContinuumTet4NodesNumbering[j];
                    int nodeID = celltype4ELementsAndNodes[nElementGroupsI - 1][i1, ren1 + 1];


                    // update info for nodes of element of outerGroup
                    nodeBeliingsInOutterAndInner[nodeID - 1, 0] = true;

                    int column = columnToWriteElemen[nodeID - 1]; columnToWriteElemen[nodeID - 1]++;

                    nodeElements[nodeID - 1, column] = elementId;
                    nodeElementsGroup[nodeID - 1, column] = nElementGroupsI - 1;
                    nodeElementsPositionInTheirGroup[nodeID - 1, column] = i1;


                }
            }


            //innerElements
            for (int i2 = 0; i2 < nElementGroupsI - 1; i2++)
            {
                for (int i1 = 0; i1 < celltype4ELementsAndNodes[i2].GetLength(0); i1++)
                {
                    int elementId = celltype4ELementsAndNodes[i2][i1, 0];
                    for (int j = 0; j < 4; j++)
                    {
                        int ren1 = ContinuumTet4NodesNumbering[j];
                        int nodeID = celltype4ELementsAndNodes[i2][i1, ren1 + 1];


                        // update info for nodes of element of inner
                        nodeBeliingsInOutterAndInner[nodeID - 1, 1] = true;

                        int column = columnToWriteElemen[nodeID - 1]; columnToWriteElemen[nodeID - 1]++;

                        nodeElements[nodeID - 1, column] = elementId;
                        nodeElementsGroup[nodeID - 1, column] = i2;
                        nodeElementsPositionInTheirGroup[nodeID - 1, column] = i1;
                    }
                }
            }

            int[] duplicatesNodeId = new int[rveNodeData.GetLength(0)];
            int newNodeIdCounter = rveNodeData.GetLength(0) + 1;
            int totalDuplicates = 0;
            bool[] hasDuplicate = new bool[rveNodeData.GetLength(0)];

            // find number of elements that should create duplicate and assigne new ids
            for (int i1 = 0; i1 < rveNodeData.GetLength(0); i1++)
            {
                bool willDuplicate = nodeBeliingsInOutterAndInner[i1, 0] && nodeBeliingsInOutterAndInner[i1, 1];
                if (willDuplicate)
                {
                    duplicatesNodeId[i1] = newNodeIdCounter;
                    newNodeIdCounter++;
                    totalDuplicates++;
                    hasDuplicate[i1] = true;
                }
            }

            // create node data for new nodes
            double[,] extraNodesCoordinates = new double[totalDuplicates, 4];
            int newNodeCounter = 0;
            for (int i1 = 0; i1 < rveNodeData.GetLength(0); i1++)
            {
                bool willDuplicate = hasDuplicate[i1];
                if (willDuplicate)
                {
                    int newNodeId = duplicatesNodeId[i1];
                    extraNodesCoordinates[newNodeCounter, 0] = newNodeId;

                    for (int i2 = 0; i2 < 3; i2++)
                    {
                        extraNodesCoordinates[newNodeCounter, 1 + i2] = rveNodeData[i1, 1 + i2];
                    }

                    newNodeCounter++;
                }


            }

            //find max element Id (of tets) to continue adding cohesive from that id
            int maxElementId = 0;
            for (int i2 = 0; i2 < nElementGroupsI; i2++)
            {
                for (int i1 = 0; i1 < celltype4ELementsAndNodes[i2].GetLength(0); i1++)
                {
                    int elId = celltype4ELementsAndNodes[i2][i1, 0];
                    if (elId > maxElementId)
                    { maxElementId = elId; }
                }
            }
            int maxNodeId = (int)rveNodeData[rveNodeData.GetLength(0) - 1, 0];

            List<int[]> cohesiveElementNodes = new List<int[]>();
            var renum = ContinuumTet4NodesNumbering;// auxilliary assignment
            //Go through all inner element faces, find if all nodes duplicated replace nodes add cohesive
            //innerElements
            int newCohesiveId = maxElementId + 1;
            for (int i2 = 0; i2 < nElementGroupsI - 1; i2++)
            {
                for (int i1 = 0; i1 < celltype4ELementsAndNodes[i2].GetLength(0); i1++)
                {
                    int elementId = celltype4ELementsAndNodes[i2][i1, 0];

                    int[,] face = new int[,] { { 1, 2, 3 }, { 1, 0, 2 }, { 2, 0, 3 }, { 1, 3, 0 } }; // element's faces local node numbering

                    for (int i3 = 0; i3 < 4; i3++)
                    {
                        bool areAllNodesDuplicated = (hasDuplicate[celltype4ELementsAndNodes[i2][i1, 1 + face[i3, 0]]-1] && hasDuplicate[celltype4ELementsAndNodes[i2][i1, 1 + face[i3, 1]] - 1]) && hasDuplicate[celltype4ELementsAndNodes[i2][i1, 1 + face[i3, 2]]-1];

                        if (areAllNodesDuplicated)
                        {
                            //add cohesive
                            int[] newCohesivesNodes = new int[] { newCohesiveId,
                                celltype4ELementsAndNodes[i2][i1, 1 + face[i3,0]], celltype4ELementsAndNodes[i2][i1, 1 + face[i3,1]], celltype4ELementsAndNodes[i2][i1, 1 + face[i3,2]],
                            duplicatesNodeId[celltype4ELementsAndNodes[i2][i1, 1 + face[i3,0]]-1], duplicatesNodeId[celltype4ELementsAndNodes[i2][i1, 1 + face[i3,1]]-1], duplicatesNodeId[celltype4ELementsAndNodes[i2][i1, 1 + face[i3,2]]-1]};
                            cohesiveElementNodes.Add(newCohesivesNodes);
                            newCohesiveId++;
                        }
                    }

                }
            }

            //convert new cohesive data list into int[,] array
            int[,] TetInterfaceCohesiveNodeIds = new int[cohesiveElementNodes.Count, 7];
            for (int i1 = 0; i1 < cohesiveElementNodes.Count; i1++)
            {
                for (int i2 = 0; i2 < 7; i2++)
                {
                    TetInterfaceCohesiveNodeIds[i1, i2] = cohesiveElementNodes[i1][i2];
                }

            }

            //update existing elements' nodes
            for (int i2 = 0; i2 < nElementGroupsI - 1; i2++)
            {
                for (int i1 = 0; i1 < celltype4ELementsAndNodes[i2].GetLength(0); i1++)
                {

                    for (int i3 = 1; i3 < celltype4ELementsAndNodes[i2].GetLength(1); i3++)
                    {
                        int nodeId = celltype4ELementsAndNodes[i2][i1, i3];
                        if (nodeId <= maxNodeId)
                        {
                            if (hasDuplicate[nodeId - 1])
                            {
                                celltype4ELementsAndNodes[i2][i1, i3] = duplicatesNodeId[nodeId - 1];
                            }
                        }
                    }


                }
            }

            bool printOutput = false;
            if (printOutput)
            {
                (new Array2DWriter()).WriteToFile(rveNodeData, @"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\rveNodeData.txt");
                (new Array2DWriter()).WriteToFile(extraNodesCoordinates, @"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\extraNodesCoordinates.txt");
                double[,] TetInterfaceCohesiveNodeIdsArray = ConvertIntArrayToDoubleArray(TetInterfaceCohesiveNodeIds);
                (new Array2DWriter()).WriteToFile(TetInterfaceCohesiveNodeIdsArray, $@"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\TetInterfaceCohesiveNodeInfo.txt");
                for (int i1 = 0; i1 < celltype4ELementsAndNodes.Count; i1++)
                {
                    var ElementAndNodes = celltype4ELementsAndNodes[i1];
                    double[,] ElAndNodesDoubleArray = ConvertIntArrayToDoubleArray(ElementAndNodes);
                    (new Array2DWriter()).WriteToFile(ElAndNodesDoubleArray, $@"C:\Users\acivi\Documents\notes_elegxoi_2\develop_nl_iga_shell\MSolve_output\gmsh_rve\solidElementNodes_{i1}.txt");
                }

            }

            return (TetInterfaceCohesiveNodeIds, extraNodesCoordinates);
        }


        public static void ReadFileOriginal1()
        {
            //string filePath = $@"C:\Users\acivi\Documents\notes_elegxoi_2\developGmsh_example\entityless\t16Solid.msh";
            //string filePath = $@"C:\Users\acivi\Documents\notes_elegxoi_2\developGmsh_example\2nd iteration\t16Solid_physical_entities.msh";//.//.
            string filePath = $@"C:\Users\acivi\Documents\notes_elegxoi_2\developGmsh_example\2nd iteration\t16Solid_physical_entities_no_volume_tag_change_More_inclusions.msh";//.//.

            var reader = new StreamReader(filePath);

            string line;
            // Find nodes segment
            while (true)
            {
                line = reader.ReadLine();
                if (line[0] == '$')
                    if (line.Equals("$Nodes")) break; // Next line will be the nodes count.
            }

            //Read the Line: numEntityBlocks(size_t) numNodes(size_t) minNodeTag(size_t) maxNodeTag(size_t)
            var firstLine = reader.ReadLine();
            string[] subStrings = firstLine.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
            int numEntityBlocks = Int32.Parse(subStrings[0]);
            int numNodes = Int32.Parse(subStrings[1]);
            int minNodeTag = Int32.Parse(subStrings[2]);
            int maxNodeTag = Int32.Parse(subStrings[3]);

            int[][] entitiesNodeIds = new int[numEntityBlocks][];
            double[][,] entitiesNodeCoords = new double[numEntityBlocks][,];
            int[] entitiesIds = new int[numEntityBlocks];
            int[] entitiesDimension = new int[numEntityBlocks];

            int numEntitities = 0;
            while (true)
            {
                line = reader.ReadLine();
                if (line == "$EndNodes")
                { break; } // Next line will be the nodes count.
                else
                {
                    numEntitities++;
                    subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    //Read the Line: entityDim(int) entityTag(int) parametric(int; 0 or 1) numNodesInBlock(size_t)
                    int entityDim = Int32.Parse(subStrings[0]); //0:Point 1:Line 2:Surface 3:Volume
                    int numTag = Int32.Parse(subStrings[1]); //p.x tag = 9
                    int isParamteric = Int32.Parse(subStrings[2]); //isParametric boolean px 0: false
                    int numNodesInBlock = Int32.Parse(subStrings[3]); //num Nodes to Follow
                    entitiesDimension[numEntitities - 1] = entityDim;

                    int[] entityNodeIds = new int[numNodesInBlock];
                    for (int i1 = 0; i1 < numNodesInBlock; i1++)
                    {
                        line = reader.ReadLine();
                        entityNodeIds[i1] = Int32.Parse(line);
                    }
                    entitiesNodeIds[numEntitities - 1] = entityNodeIds;

                    double[,] entityNodeCoords = new double[numNodesInBlock, 3];
                    for (int i1 = 0; i1 < numNodesInBlock; i1++)
                    {
                        line = reader.ReadLine();
                        subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                        entityNodeCoords[i1, 0] = Double.Parse(subStrings[0]);
                        entityNodeCoords[i1, 1] = Double.Parse(subStrings[1]);
                        entityNodeCoords[i1, 2] = Double.Parse(subStrings[2]);
                    }
                    entitiesNodeCoords[numEntitities - 1] = entityNodeCoords;
                    entitiesIds[numEntitities - 1] = numTag;

                }

            }





            // Find elements segment
            while (true)
            {
                line = reader.ReadLine();
                if (line[0] == '$')
                    if (line.Equals("$Elements")) break; // Next line will be the nodes count.
            }
            var firstElementInfoLine = reader.ReadLine();
            subStrings = firstElementInfoLine.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);

            int numElemEntityBlocks = Int32.Parse(subStrings[0]);
            int numElements = Int32.Parse(subStrings[1]);

            int minElementTag = Int32.Parse(subStrings[2]);
            int maxElementTag = Int32.Parse(subStrings[3]);

            int[][] elementIdsOfEachBlock1 = new int[numElemEntityBlocks][];
            int[] elementBlocksTags = new int[numElemEntityBlocks];
            int[][,] elementNodesIdsOfEachBlock1 = new int[numElemEntityBlocks][,];
            int[] elementBlocksCellTypes = new int[numElemEntityBlocks];


            int numElementBlocks1 = 0;
            while (true)
            {
                line = reader.ReadLine();
                if (line == "$EndElements")
                { break; } // Next line will be the nodes count.
                else
                {
                    numElementBlocks1++;
                    subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    //Read the Line: entityDim(int) entityTag(int) parametric(int; 0 or 1) numNodesInBlock(size_t)
                    int entityDim = Int32.Parse(subStrings[0]); //0:Point 1:Line 2:Surface 3:Volume
                    int entityTag = Int32.Parse(subStrings[1]); //p.x tag = 9
                    elementBlocksTags[numElementBlocks1 - 1] = entityTag;
                    int elementType = Int32.Parse(subStrings[2]); // einai o arithmos gia to celltype pou xrhsimopoieitai.
                    elementBlocksCellTypes[numElementBlocks1 - 1] = elementType;
                    int numElementsInBlock = Int32.Parse(subStrings[3]); //num Nodes to Follow


                    int[] blockElementIds = new int[numElementsInBlock];
                    line = reader.ReadLine();
                    subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                    int[,] entityElementNodesIds = new int[numElementsInBlock, subStrings.Length - 1];
                    blockElementIds[0] = Int32.Parse(subStrings[0]);//.
                    for (int i1 = 0; i1 < subStrings.Length - 1; i1++)
                    {
                        entityElementNodesIds[0, i1] = Int32.Parse(subStrings[i1 + 1]);
                    }

                    for (int i2 = 1; i2 < numElementsInBlock; i2++)
                    {
                        line = reader.ReadLine();
                        subStrings = line.Split(new char[] { ' ', ',' }, StringSplitOptions.RemoveEmptyEntries);
                        blockElementIds[i2] = Int32.Parse(subStrings[0]);//.
                        for (int i1 = 0; i1 < subStrings.Length - 1; i1++)
                        {
                            entityElementNodesIds[i2, i1] = Int32.Parse(subStrings[i1 + 1]);
                        }
                    }
                    elementIdsOfEachBlock1[numElementBlocks1 - 1] = blockElementIds;
                    elementNodesIdsOfEachBlock1[numElementBlocks1 - 1] = entityElementNodesIds;

                }

            }

        }

        public static double[,] ConvertIntArrayToDoubleArray (int[,] data)
        {
            double[,] dataDouble = new double[data.GetLength(0), data.GetLength(1)];

            for (int i1 = 0; i1 < data.GetLength(0); i1++)
            {
                for (int i2 = 0; i2 < data.GetLength(1); i2++)
                {
                    dataDouble[i1, i2] = (double)data[i1, i2];
                }
            }

            return dataDouble;
        }
    }
}
