using System;
using System.Runtime.InteropServices;
using Ijw.Math;

namespace Ijw.Physics
{
	public delegate void BodyEventHandler();
	public delegate void CollisionEventHandler(Body other);

	#region P/Invoke
	[UnmanagedFunctionPointer(CallingConvention.Cdecl)]
	delegate void NewtonBodyEventHandler(int body);
	#endregion

	public sealed class Body : IDisposable
	{
		readonly World world;
		readonly int id;

		float mass;
		Vector3 moments;
		readonly object owner;
		CollisionBehavior collisionBehavior;
		bool autoFreeze;
		float linearDamping;
		Vector3 angularDamping;
		float staticFriction;
		float kineticFriction;
		Collider collider;

		public Collider Collider
		{
			get { return collider; }
			set
			{
				collider = value;
				NewtonBodySetCollision(id, value.Id);
			}
		}

		public float StaticFriction
		{
			get { return staticFriction; }
			set { staticFriction = value; }
		}

		public float KineticFriction
		{
			get { return kineticFriction; }
			set { kineticFriction = value; }
		}

		public float LinearDamping
		{
			get { return linearDamping; }
			set
			{
				linearDamping = value;
				NewtonBodySetLinearDamping(id, linearDamping);
			}
		}

		public Vector3 AngularDamping
		{
			get { return angularDamping; }
			set
			{
				angularDamping = value;
				NewtonBodySetAngularDamping(id, ref value);
			}
		}

		public event BodyEventHandler OnGetForces;
		public event BodyEventHandler OnMatrixChanged;
		public event CollisionEventHandler OnCollided;

		internal int Id
		{
			get { return id; }
		}

		public float Mass
		{
			get { return mass; }
			set
			{
				mass = value;
				CommitMassMatrix();
			}
		}

		public Vector3 Moments
		{
			get { return moments; }
			set
			{
				moments = value;
				CommitMassMatrix();
			}
		}

		public object Owner
		{
			get { return owner; }
		}

		public CollisionBehavior CollisionBehavior
		{
			get { return collisionBehavior; }
			set { collisionBehavior = value; }
		}

		public bool AutoFreeze
		{
			get { return autoFreeze; }
			set
			{
				autoFreeze = value;
				NewtonBodySetAutoFreeze(id, autoFreeze ? 1 : 0);
			}
		}

		public Matrix Matrix
		{
			get
			{
				Matrix matrix = new Matrix();
				NewtonBodyGetMatrix(id, out matrix);
				return matrix;
			}

			set
			{
				NewtonBodySetMatrix(id, ref value);
			}
		}

		void CommitMassMatrix()
		{
			NewtonBodySetMassMatrix(id, mass, moments.x, moments.y, moments.z);
		}

		public Vector3 Velocity
		{
			get
			{
				Vector3 result;
				NewtonBodyGetVelocity(id, out result);
				return result;
			}

			set
			{
				NewtonBodySetVelocity(id, ref value);
			}
		}

		internal void DispatchCollided(Body other)
		{
			if (OnCollided != null)
				OnCollided(other);
		}

		public Body(World world, object owner, Matrix matrix, Collider collider)
		{
			this.collider = collider;
			this.world = world;
			id = NewtonCreateBody(world.Id, collider.Id);
			this.owner = owner;
			world.BodyCreated(id, this);

			setTransform = delegate(int bodyId)
			{
				Body body = world.GetBody(bodyId);
				if (body == null)
					return;

				if (body.OnMatrixChanged != null)
					body.OnMatrixChanged();
			};

			NewtonBodySetTransformCallback(id, setTransform);

			getForces = delegate(int bodyId)
			{
				Body body = world.GetBody(bodyId);
				if (body == null)
					return;

				if (body.OnGetForces != null)
					body.OnGetForces();
			};

			NewtonBodySetForceAndTorqueCallback(id, getForces);

			Matrix = matrix;
		}

		bool disposed = false;
		public void Dispose()
		{
			if (disposed)
				return;

			disposed = true;
			GC.SuppressFinalize(this);
			NewtonDestroyBody(world.Id, id);
			world.BodyDisposed(id);
		}

		bool continuous = false;
		public bool EnableContinuousCollision
		{
			get { return continuous; }
			set
			{
				continuous = value;
				NewtonBodySetContinuousCollisionMode(id, value ? 1 : 0);
			}
		}

		readonly NewtonBodyEventHandler setTransform;
		readonly NewtonBodyEventHandler getForces;

		~Body()
		{
			Dispose();
		}

		#region P/Invoke

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern int NewtonCreateBody(
			int world,
			int collider);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonDestroyBody(
			int world,
			int body);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetTransformCallback(int body, NewtonBodyEventHandler p);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetForceAndTorqueCallback(int body, NewtonBodyEventHandler p);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetMassMatrix(
			int bodyId,
			float mass,
			float tx,
			float ty,
			float tz);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetAutoFreeze(
			int bodyId,
			int allowFreeze);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetLinearDamping(
			int bodyId,
			float value);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetAngularDamping(
			int bodyId,
			[In] ref Vector3 damping);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodyGetMatrix( int bodyId, out Matrix matrix);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetMatrix( int bodyId, [In] ref Matrix matrix);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetForce( int bodyId, [In] ref Vector3 force);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodyGetVelocity(int bodyId, out Vector3 velocity);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetVelocity(int bodyId, [In] ref Vector3 velocity);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetContinuousCollisionMode(int bodyId, int value);

		[DllImport("Newton.dll", CallingConvention = CallingConvention.Cdecl)]
		static extern void NewtonBodySetCollision(int bodyId, int collisionId);

		#endregion

		public void SetForce(Vector3 force)
		{
			NewtonBodySetForce(id, ref force);
		}
	}
}
