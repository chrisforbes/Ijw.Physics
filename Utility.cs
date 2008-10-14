using System;
using System.Collections.Generic;
using System.Text;
using Ijw.Math;

namespace Ijw.Physics
{
	public static class Utility
	{
		static Vector3 BaseValues(Vector3 extents)
		{
			return new Vector3(
				extents.y * extents.y + extents.z * extents.z,
				extents.x * extents.x + extents.z * extents.z,
				extents.x * extents.x + extents.y * extents.y
			);
		}

		public static Vector3 MomentsForSolidSphere( Vector3 radii, float mass )
		{
			return mass / 5.0f * BaseValues(radii);
		}

		public static Vector3 MomentsForHollowSphere(Vector3 radii, float mass)
		{
			return mass / 3.0f * BaseValues(radii);
		}

		public static Vector3 MomentsForSolidBox(Vector3 size, float mass)
		{
			return mass / 12.0f * BaseValues(size);
		}

		public static Vector3 MomentsForSolidSphere(float radius, float mass)
		{
			return MomentsForSolidSphere(new Vector3(radius, radius, radius), mass);
		}
	}
}
