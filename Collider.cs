using System;
using System.Collections.Generic;
using System.Text;
using System.Runtime.InteropServices;
using Ijw.Math;
using IjwFramework.Types;

namespace Ijw.Physics
{
	public class Collider : IDisposable
	{
		readonly World world;
		readonly int id;

		Collider(int id, World world)
		{
			this.id = id;
			this.world = world;
		}

		internal int Id
		{
			get { return id; }
		}

		public static Collider CreateSphere(World world, Vector3 size)
		{
			return new Collider(NewtonCreateSphere(world.Id, size.x, size.y, size.z, 0), world);
		}

		public static Collider CreateBox(World world, Vector3 size)
		{
			return new Collider(NewtonCreateBox(world.Id, size.x, size.y, size.z, 0), world);
		}

/*		public static Collider CreateMesh(World world, params Pair<Model, Matrix>[] meshes)
		{
			int colliderId = NewtonCreateTreeCollision(world.Id, 0);
			NewtonTreeCollisionBeginBuild(colliderId);

			foreach (Pair<Model, Matrix> m in meshes)
				Add(colliderId, m);

			NewtonTreeCollisionEndBuild(colliderId, false);
			return new Collider(colliderId, world);
		}

		static void Add(int id, Pair<Model, Matrix> model)
		{
			Vector3[] vertices = model.first.VertexPositions;
			short[] indices = model.first.Indices;

			Vector3[] face = new Vector3[3];
			for (int i = 0; i < indices.Length / 3; i++)
			{
				for (int j = 0; j < 3; j++)
					face[j] = vertices[indices[i * 3 + j]].TransformAsCoordinate(model.second);
				NewtonTreeCollisionAddFace(id, 3, face, 12, 0);
			}
		}	*/

		#region P/Invoke

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern int NewtonCreateSphere(int world, float dx, float dy, float dz, int unused);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern int NewtonCreateBox(int world, float dx, float dy, float dz, int unused);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern int NewtonCreateTreeCollision(int world, int sbz);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonTreeCollisionBeginBuild(int colliderId);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonTreeCollisionEndBuild(int colliderId, bool optimize);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonTreeCollisionAddFace(int colliderId,
			int vertexCount,
			[In][MarshalAs(UnmanagedType.LPArray, SizeConst = 3)]Vector3[] tri,
			int vertexStride,
			int vertexAttrib);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		internal static extern void NewtonReleaseCollision( int world, int collider);

		#endregion

		~Collider()
		{
			Dispose();
		}

		bool disposed = false;
		public void Dispose()
		{
			if (disposed)
				return;

			disposed = true;
			GC.SuppressFinalize(this);

			NewtonReleaseCollision(world.Id, Id); 
		}
	}
}
