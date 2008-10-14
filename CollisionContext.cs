using System;
using System.Collections.Generic;

namespace Ijw.Physics
{
	public sealed class CollisionContext
	{
		readonly Body one, two;

		public CollisionContext(Body one, Body two)
		{
			this.one = one;
			this.two = two;
		}

		bool hasContacts = false;

		public void AddContact()
		{
			hasContacts = true;
		}

		public float StaticFriction
		{
			get
			{
				if (one == null || two == null)
					return 1.0f;

				return one.StaticFriction * two.StaticFriction;
			}
		}

		public float KineticFriction
		{
			get
			{
				if (one == null || two == null)
					return 1.0f;

				return one.KineticFriction * two.KineticFriction;
			}
		}

		public CollisionBehavior CollisionBehavior
		{
			get
			{
				if (one == null || two == null)
					return CollisionBehavior.NotSolid;

				if (one.CollisionBehavior != two.CollisionBehavior)
					return CollisionBehavior.NotSolid;
				return one.CollisionBehavior;
			}
		}

		public bool HasContacts
		{
			get { return hasContacts; }
		}

		public void DispatchCollided()
		{
			if (one == null || two == null)
				return;

			one.DispatchCollided(two);
			two.DispatchCollided(one);
		}
	}
}
