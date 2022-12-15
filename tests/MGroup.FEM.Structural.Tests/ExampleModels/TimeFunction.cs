using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MGroup.FEM.Structural.Tests.ExampleModels
{
	public class TimeFunctionSinusoidalSum
	{
		private double[] omegas ;

		private double[] amplitudes;

		public TimeFunctionSinusoidalSum(double[] omegas, double[] amplitudes)
		{
			this.omegas = omegas;
			this.amplitudes = amplitudes;
		}

		public double TimeFunctionForData(double t, double amount)
		{
			double endiameshTimeFuncValue = 0;
            for (int i1 = 0; i1 < omegas.Length; i1++)
            {
				endiameshTimeFuncValue+= amplitudes[i1] * Math.Sin(omegas[i1] * t);

			}
			return endiameshTimeFuncValue * amount;
			

			
		}
	}

	public class TimeFunctionPeriodiTrianglePulse
	{
		private double[] time;

		private double[] timeFunctionValues;

		public TimeFunctionPeriodiTrianglePulse(double[] timeIntervals, double[] timeFunctionValues)
        {
			this.time = timeIntervals;
			this.timeFunctionValues = timeFunctionValues;
        }


		public  double TimeFunctionPeriodicLoad(double t, double amount)
		{
			for (int i1 = 0; i1 < time.Length - 1; i1++)
			{
				if (t == 0)
				{
					double endiameshTimeFuncValue = time[0];
					return endiameshTimeFuncValue * amount;
				}
				if ((time[i1] < t) && (t <= time[i1 + 1]))
				{
					double klisi = (timeFunctionValues[i1 + 1] - timeFunctionValues[i1]) / (time[i1 + 1] - time[i1]);
					double dt = t - time[i1];
					double deltaTimeFunction = klisi * dt;
					double endiameshTimeFuncValue = timeFunctionValues[i1] + deltaTimeFunction;
					return endiameshTimeFuncValue * amount;

				}

			}

			throw new Exception("time out of range");
		}
	}
}
