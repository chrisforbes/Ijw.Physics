using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using Ijw.Math;

namespace Ijw.Physics
{
	public sealed class World : IDisposable
	{
		readonly int id;
		internal int Id
		{
			get { return id; }
		}

		bool disposed = false;

		~World()
		{
			Dispose();
		}

		public void Dispose()
		{
			if (disposed)
				return;

			disposed = true;
			GC.SuppressFinalize(this);

			foreach (Body body in new List<Body>(bodies.Values))
				body.Dispose();
			bodies.Clear();

			NewtonDestroy(id);
		}

		public World()
		{
			id = NewtonCreate(0, 0);
			int group = NewtonMaterialGetDefaultGroupID(id);

			begin = delegate(int material, int first, int second)
			{
				context = new CollisionContext(GetBody(first), GetBody(second));
				return 1;
			};

			process = delegate(int material, int contact)
			{
				context.AddContact();
				return context.CollisionBehavior == CollisionBehavior.Solid ? 1 : 0;
			};

			end = delegate(int material)
			{
				if (context.HasContacts)
					context.DispatchCollided();

				context = null;
			};

			NewtonMaterialSetCollisionCallback(id, group, group, 0, begin, process, end);
			NewtonMaterialSetContinuousCollisionMode(id, group, group, 1);
		}

		Dictionary<int, Body> bodies = new Dictionary<int, Body>();
		internal Body GetBody(int id)
		{
			Body body;
			return bodies.TryGetValue(id, out body) ? body : null;
		}

		internal void BodyDisposed(int id)
		{
			bodies.Remove(id);
		}

		internal void BodyCreated(int id, Body body)
		{
			bodies.Add(id, body);
		}

		double lastTime = 0;

		public void ResetTime(double time)
		{
			lastTime = time;
		}

		public void Update(double time)
		{
			const float physicsTimeStep = 1.0f / 256.0f;

			while (lastTime < time)
			{
				float stepSize = physicsTimeStep;
				NewtonUpdate(id, stepSize);
				lastTime += stepSize;
			}
		}

		public void SetBounds(Vector3 maxDisplacement)
		{
			Vector3 min = -maxDisplacement;
			NewtonSetWorldSize(id, ref min, ref maxDisplacement);
		}

		readonly ContactBegin begin;
		readonly ContactProcess process;
		readonly ContactEnd end;

		CollisionContext context;

		#region P/Invoke

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern int NewtonCreate(int malloc, int free);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonDestroy(int world);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonUpdate(int worldId, float stepSize);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern int NewtonMaterialGetDefaultGroupID(int worldId);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonMaterialSetCollisionCallback(
			int worldId,
			int c0, int c1,
			int sbz,
			ContactBegin ContactBeginCallback,
			ContactProcess ContactProcessCallback,
			ContactEnd ContactEndCallback);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonSetWorldSize(
			int world, [In] ref Vector3 min, [In] ref Vector3 max);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonMaterialSetContinuousCollisionMode(
			int world, int c0, int c1, int state);

		#endregion
	}

	#region P/Invoke

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	delegate int ContactBegin(int material, int body0, int body1);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	delegate int ContactProcess(int material, int contact);

	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	delegate void ContactEnd(int material);

	#endregion
}
