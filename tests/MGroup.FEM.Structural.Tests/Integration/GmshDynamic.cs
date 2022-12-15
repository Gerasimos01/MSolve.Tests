using System.Collections.Generic;
using MGroup.Constitutive.Structural;
using MGroup.MSolve.Discretization.Entities;
using MGroup.MSolve.DataStructures;
using MGroup.MSolve.Discretization.Dofs;
using MGroup.MSolve.Solution;
using MGroup.NumericalAnalyzers;
using MGroup.NumericalAnalyzers.Logging;
using MGroup.NumericalAnalyzers.Discretization.NonLinear;
using MGroup.Solvers.Direct;
using MGroup.FEM.Structural.Tests.ExampleModels;
using MGroup.FEM.Structural.Tests.Commons;
using Xunit;
//using modelBuilder = MGroup.FEM.Structural.Tests.ExampleModels.GmshCompositeRveBuilder;
using System.Linq;
using MGroup.Constitutive.Structural.BoundaryConditions;
using MGroup.MSolve.Discretization.BoundaryConditions;
using MGroup.Solvers.DofOrdering;
using MGroup.Solvers.DofOrdering.Reordering;
using MGroup.NumericalAnalyzers.Dynamic;
using System;

namespace MGroup.FEM.Structural.Tests.Integration
{
	public static class GmshDynamic
	{   // ORIGIN: PlateRevisitedLessConstrainted.cs from Preliminary2_Copy repo
		// changes: updated to new msolve and for dynamic problem implementation

		//Edw epilegontai oi parametroi ts analushs
		public static int NR_steps = 1;
        private static double timestep=0.0005; // those are overwritten for the periodic example
        private static double totalTime=0.08; // those are overwritten for the periodic example
		private static GmshCompositeRveBuilder modelBuilder;

		//Oi parametroi tou mondelou kai twn constrains  allazoun sto modelBuilder dld PlateRevisitedLessConstrainted

		[Fact]
		private static void RunSuddenLoadTest()
		{
			modelBuilder = new GmshCompositeRveBuilder("..\\..\\..\\InputFiles\\t16Solid_physical_entities_no_volume_tag_change_More_inclusions.msh");
			Model model = modelBuilder.GetModel();

			modelBuilder.AddStaticNodalLoads(model);

			(double[] computedDisplacements, double[][] freeDofsDisplacementsPerIncremen1) = SolveModelDynamic(model);

		}

		
		[Fact]
		private static void RunTransientTestPeriodicObject1()
		{
		    timestep = 0.0005; totalTime = 0.16;

			int epiluseis = 1;

            for (int i = 0; i < epiluseis; i++)
            {
				double[] omegas = new double[] { 20, 80, 200, 480 };
				double[] amplitudes = new double[] { 1, 1, 1, 1 };

				var timeProvider = new TimeFunctionSinusoidalSum(omegas, amplitudes);
				//modelBuilder.monitoredDof = StructuralDof.TranslationX;

				modelBuilder = new GmshCompositeRveBuilder("..\\..\\..\\InputFiles\\t16Solid_physical_entities_no_volume_tag_change_More_inclusions.msh");
				Model model = modelBuilder.GetModel();

				modelBuilder.AddPeriodicTransientLoadTimeFunc(model, timeProvider.TimeFunctionForData);

				(double[] computedDisplacements, double[][] freeDofsDisplacementsPerIncremen1) = SolveModelDynamic(model);

                for (int i1 = 0; i1 < freeDofsDisplacementsPerIncremen1.Length; i1++)
                {
					var path = $@"C:\Users\Public\Documents\MSolve_output\totalTisplacemnts_AnalysisNo_{i}_TimeStep_{i1 + 1}.txt";
					(new MGroup.LinearAlgebra.Output.Array1DWriter()).WriteToFile(freeDofsDisplacementsPerIncremen1[i1], path);
				}

			}

			
			timestep = 0.0005;totalTime = 0.08;
			//Assert.True(Utilities.AreDisplacementsSame(modelBuilder.GetExpectedDisplacementsPeriodicLoadADINA(),
																//computedDisplacements, tolerance: 1e-5));
		}
		private static (double[] totalDisplacementOverTimeDOFtade, double[][] freeDofsDisplacementsPerIncremen1) SolveModelDynamic(Model model)
		{
			var solverFactory = new SuiteSparseSolver.Factory() { DofOrderer = new DofOrderer(new NodeMajorDofOrderingStrategy(), new NodeMajorReordering()) };
			var algebraicModel = solverFactory.BuildAlgebraicModel(model);
			var solver = solverFactory.BuildSolver(algebraicModel);
			var problem = new ProblemStructural(model, algebraicModel);

			var loadControlAnalyzerBuilder = new LoadControlAnalyzer.Builder( algebraicModel, solver, problem, numIncrements: NR_steps)
			{
				ResidualTolerance = 1E-10,
				MaxIterationsPerIncrement = 100,
				NumIterationsForMatrixRebuild = 1
			};
			var loadControlAnalyzer = loadControlAnalyzerBuilder.Build();


			//var staticAnalyzer = new StaticAnalyzer(model, algebraicModel, problem, loadControlAnalyzer);
			var dynamicAnalyzerBuilder = new NewmarkDynamicAnalyzer.Builder( algebraicModel, problem, loadControlAnalyzer,
				timeStep: timestep, totalTime: totalTime);
			dynamicAnalyzerBuilder.SetNewmarkParametersForConstantAcceleration();
			NewmarkDynamicAnalyzer parentAnalyzer = dynamicAnalyzerBuilder.Build();
			parentAnalyzer.ResultStorage = new ImplicitIntegrationAnalyzerLog();



			//logs 
			var node_A = modelBuilder.monitoredNodeAId;

			var emptyBCs = new List<INodalBoundaryCondition>();


			var loggerA = new TotalLoadsDisplacementsPerIncrementLog(model.NodesDictionary[node_A], modelBuilder.loadedDof, emptyBCs, algebraicModel,
				$"hexaContinuumCantileverDynamicResults.txt");
			

			loadControlAnalyzer.IncrementalLog = loggerA;
			List<(INode node, IDofType dof)> watchDofs = new List<(INode node, IDofType dof)>();
			watchDofs.Add((model.NodesDictionary[node_A], modelBuilder.loadedDof));
			loadControlAnalyzer.LogFactory = new LinearAnalyzerLogFactory(watchDofs, algebraicModel);
			
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			int totalNewmarkstepsNum = (int)Math.Truncate(totalTime / timestep);
			var totalDisplacementOverTime = new double[totalNewmarkstepsNum];
			for (int i1 = 0; i1 < totalNewmarkstepsNum; i1++)
            {
				var timeStepResultsLog = parentAnalyzer.ResultStorage.Logs[i1];
				totalDisplacementOverTime[i1] = ((DOFSLog)timeStepResultsLog).DOFValues[model.GetNode(node_A), modelBuilder.loadedDof];
			}

			var freeDofsDisplacementsPerIncremen = parentAnalyzer.DisplacementsPerTimeStep;

			double[][] freeDofsDisplacementsPerIncremen1 = new double[freeDofsDisplacementsPerIncremen.Length][];

            for (int i1 = 0; i1 < freeDofsDisplacementsPerIncremen.Length; i1++)
            {
				freeDofsDisplacementsPerIncremen1[i1] = algebraicModel.CheckCompatibleVector(freeDofsDisplacementsPerIncremen[i1]).SingleVector.CopyToArray();

			}



			return (totalDisplacementOverTime, freeDofsDisplacementsPerIncremen1);



		}
	}

}
