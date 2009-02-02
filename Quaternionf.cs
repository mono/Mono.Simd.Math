// Quaternionf.cs
//
// Authors:
//   Jerry Maine (crashfourit@gmail.com)
//   John Hurliman (john.hurliman@intel.com)
//
// (C) Jerry Maine & Intel Corporation
//
// Permission is hereby granted, free of charge, to any person obtaining
// a copy of this software and associated documentation files (the
// "Software"), to deal in the Software without restriction, including
// without limitation the rights to use, copy, modify, merge, publish,
// distribute, sublicense, and/or sell copies of the Software, and to
// permit persons to whom the Software is furnished to do so, subject to
// the following conditions:
//
// The above copyright notice and this permission notice shall be
// included in all copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
// EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
// MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
// LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
// OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
// WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//

using System;
using System.Runtime.InteropServices;

namespace Mono.Simd.Math
{
    [Serializable]
    [StructLayout(LayoutKind.Explicit, Pack = 0, Size = 16)]
    public struct Quaternionf
    {
        [FieldOffset(0)]
        internal Vector4f Vector;

        #region Properties

        public float X { get { return Vector.X; } set { Vector.X = value; } }
        public float Y { get { return Vector.Y; } set { Vector.Y = value; } }
        public float Z { get { return Vector.Z; } set { Vector.Z = value; } }
        public float W { get { return Vector.W; } set { Vector.W = value; } }

        #endregion Properties

        #region Constructors

        public Quaternionf(float x, float y, float z, float w)
        {
            Vector = new Vector4f(x, y, z, w);
        }

        public Quaternionf(Vector4f vector)
        {
            Vector = vector;
        }

        public Quaternionf(Vector3f vector, float scalar)
        {
            Vector = new Vector4f(vector.X, vector.Y, vector.Z, scalar);
        }

        public Quaternionf(Quaternionf quaternion)
        {
            Vector = quaternion.Vector;
        }

		/* FromRotationMatrix
		 */
		public void Quaternionf(Matrix4f m)
        {
            float trace = m.Trace();

            if (trace > Single.Epsilon)
            {
                float s = (float)System.Math.Sqrt(trace + 1f);
                Vector.W = s * 0.5f;
                s = 0.5f / s;
                Vector.X = (m.M23 - m.M32) * s;
                Vector.Y = (m.M31 - m.M13) * s;
                Vector.Z = (m.M12 - m.M21) * s;
            }
            else
            {
                if (m.M11 > m.M22 && m.M11 > m.M33)
                {
                    float s = (float)System.Math.Sqrt(1f + m.M11 - m.M22 - m.M33);
                    Vector.X = 0.5f * s;
                    s = 0.5f / s;
                    Vector.Y = (m.M12 + m.M21) * s;
                    Vector.Z = (m.M13 + m.M31) * s;
                    Vector.W = (m.M23 - m.M32) * s;
                }
                else if (m.M22 > m.M33)
                {
                    float s = (float)System.Math.Sqrt(1f + m.M22 - m.M11 - m.M33);
                    Vector.Y = 0.5f * s;
                    s = 0.5f / s;
                    Vector.X = (m.M21 + m.M12) * s;
                    Vector.Z = (m.M32 + m.M23) * s;
                    Vector.W = (m.M31 - m.M13) * s;
                }
                else
                {
                    float s = (float)System.Math.Sqrt(1f + m.M33 - m.M11 - m.M22);
                    Vector.Z = 0.5f * s;
                    s = 0.5f / s;
                    Vector.X = (m.M31 + m.M13) * s;
                    Vector.Y = (m.M32 + m.M23) * s;
                    Vector.W = (m.M12 - m.M21) * s;
                }
            }
        }
        #endregion Constructors

        #region Public Methods

        public Vector4f Negate()
        {
            return Vector * Vector4f.MinusOne;
        }

        public void Normalize()
        {
            Vector4f factor = Vector * Vector;
            factor = factor.HorizontalAdd(factor);
            factor = factor.HorizontalAdd(factor);
            factor = factor.InvSqrt();
            Vector *= factor;
        }

        public float Length()
        {
            Vector4f length = Vector * Vector;
            length = length.HorizontalAdd(length);
            length = length.HorizontalAdd(length);
            return length.Sqrt().X;
        }

        public float LengthSquared()
        {
            Vector4f length = Vector * Vector;
            length = length.HorizontalAdd(length);
            return length.HorizontalAdd(length).X;
        }

        /// <summary>
        /// Convert this quaternion to euler angles
        /// </summary>
        /// <param name="roll">X euler angle</param>
        /// <param name="pitch">Y euler angle</param>
        /// <param name="yaw">Z euler angle</param>
        public void GetEulerAngles(out float roll, out float pitch, out float yaw)
        {
            Vector4f sq = Vector * Vector;

            // Unit will be a correction factor if the quaternion is not normalized
            Vector4f sqSum = sq.HorizontalAdd(sq);
            sqSum = sq.HorizontalAdd(sq);
            float unit = sqSum.X;

            double test = X * Y + Z * W;

            if (test > 0.499f * unit)
            {
                // Singularity at north pole
                yaw = 2f * (float)System.Math.Atan2(X, W);
                pitch = Utils.Pi / 2f;
                roll = 0f;
            }
            else if (test < -0.499f * unit)
            {
                // Singularity at south pole
                yaw = -2f * (float)System.Math.Atan2(X, W);
                pitch = -Utils.Pi / 2f;
                roll = 0f;
            }
            else
            {
                yaw = (float)System.Math.Atan2(2f * Y * W - 2f * X * Z, sq.X - sq.Y - sq.Z + sq.W);
                pitch = (float)System.Math.Asin(2f * test / unit);
                roll = (float)System.Math.Atan2(2f * X * W - 2f * Y * Z, -sq.X + sq.Y - sq.Z + sq.W);
            }
        }

        /// <summary>
        /// Convert this quaternion to an angle around an axis
        /// </summary>
        /// <param name="axis">Unit vector describing the axis</param>
        /// <param name="angle">Angle around the axis, in radians</param>
        public void GetAxisAngle(out Vector3f axis, out float angle)
        {
            Vector4f abs = Vector & (Vector4f)new Vector4i(0x7fffffff);
            abs = abs.HorizontalAdd(abs);
            abs = abs.HorizontalAdd(abs);
            float scale = abs.X;

            if (scale < Single.Epsilon || W > 1.0f || W < -1.0f)
            {
                angle = 0.0f;
                axis.X = 0.0f;
                axis.Y = 1.0f;
                axis.Z = 0.0f;
            }
            else
            {
                angle = 2.0f * (float)System.Math.Acos(W);
                float ooscale = 1f / scale;
                axis.X = X * ooscale;
                axis.Y = Y * ooscale;
                axis.Z = Z * ooscale;
            }
        }

        /// <summary>
        /// Sets this quaternion to the conjugate (spatial inverse) of itself
        /// </summary>
        public void Conjugate()
        {
            Vector.X = -X;
            Vector.Y = -Y;
            Vector.Z = -Z;
        }

        /// <summary>
        /// Conjugates and renormalizes the quaternion
        /// </summary>
        public Vector4f Invert()
        {
            float norm = LengthSquared();

            if (norm == 0f)
            {
                return Vector4f.Zero;
            }
            else
            {
                Conjugate();

                Vector4f r = new Vector4f(norm);
                r = r.Reciprocal();
                return Vector * r;
            }
        }

        /// <summary>
        /// Set this quaternion from a normalized axis and an angle of rotation
        /// around that axis
        /// </summary>
        /// <param name="axis">Axis of rotation</param>
        /// <param name="angle">Angle of rotation</param>
        public void FromAxisAngle(Vector3f axis, float angle)
        {
            angle *= 0.5f;
            float c = (float)System.Math.Cos(angle);
            float s = (float)System.Math.Sin(angle);

            Vector.X = axis.X * s;
            Vector.Y = axis.Y * s;
            Vector.Z = axis.Z * s;
            Vector.W = c;

            Vector.Normalize();
        }

        /// <summary>
        /// Set this quaternion from roll, pitch, and yaw euler angles in
        /// radians
        /// </summary>
        /// <param name="roll">X angle in radians</param>
        /// <param name="pitch">Y angle in radians</param>
        /// <param name="yaw">Z angle in radians</param>
        public void FromEulers(float roll, float pitch, float yaw)
        {
            double atCos = System.Math.Cos(roll / 2f);
            double atSin = System.Math.Sin(roll / 2f);
            double leftCos = System.Math.Cos(pitch / 2f);
            double leftSin = System.Math.Sin(pitch / 2f);
            double upCos = System.Math.Cos(yaw / 2f);
            double upSin = System.Math.Sin(yaw / 2f);
            double atLeftCos = atCos * leftCos;
            double atLeftSin = atSin * leftSin;

            Vector.X = (float)(atSin * leftCos * upCos + atCos * leftSin * upSin);
            Vector.Y = (float)(atCos * leftSin * upCos - atSin * leftCos * upSin);
            Vector.Z = (float)(atLeftCos * upSin + atLeftSin * upCos);
            Vector.W = (float)(atLeftCos * upCos - atLeftSin * upSin);
        }

        

        public float Dot(Quaternionf quaternion)
        {
            return
                (Vector.X * quaternion.Vector.X) +
                (Vector.Y * quaternion.Vector.Y) +
                (Vector.Z * quaternion.Vector.Z) +
                (Vector.W * quaternion.Vector.W);
        }

        /// <summary>
        /// Spherical linear interpolation between this quaternion and another
        /// </summary>
        /// <param name="quaternion"></param>
        /// <param name="amount"></param>
        public Quaternionf Slerp(Quaternionf quaternion, float amount)
        {
            float angle = Dot(quaternion);
            Quaternionf q1 = this;

            if (angle < 0f)
            {
                q1.Negate();
                angle *= -1f;
            }

            float scale;
            float invscale;

            if ((angle + 1f) > 0.05f)
            {
                if ((1f - angle) >= 0.05f)
                {
                    // slerp
                    float theta = (float)System.Math.Acos(angle);
                    float invsintheta = 1f / (float)System.Math.Sin(theta);
                    scale = (float)System.Math.Sin(theta * (1f - amount)) * invsintheta;
                    invscale = (float)System.Math.Sin(theta * amount) * invsintheta;
                }
                else
                {
                    // lerp
                    scale = 1f - amount;
                    invscale = amount;
                }
            }
            else
            {
                quaternion.X = -q1.Y;
                quaternion.Y = q1.X;
                quaternion.Z = -q1.W;
                quaternion.W = q1.Z;

                scale = (float)System.Math.Sin(Utils.Pi * (0.5f - amount));
                invscale = (float)System.Math.Sin(Utils.Pi * amount);
            }

            return (q1 * scale) + (quaternion * invscale);
        }

        public bool ApproxEquals(Quaternionf quaternion, float tolerance)
        {
            Vector4f diff = Vector - quaternion.Vector;
            return (diff.Length() <= tolerance);
        }

        public bool IsFinite()
        {
            return Utils.IsFinite(Vector.X) && Utils.IsFinite(Vector.Y) &&
                Utils.IsFinite(Vector.Z) && Utils.IsFinite(Vector.W);
        }

        #endregion Public Methods

        #region Overrides

        public override int GetHashCode()
        {
            return Vector.X.GetHashCode() ^ Vector.Y.GetHashCode() ^
                Vector.Z.GetHashCode() ^ Vector.W.GetHashCode();
        }

        public override string ToString()
        {
            return "<" + X + ", " + Y + ", " + Z + ", " + W + ">";
        }

        #endregion Overrides

        #region Operators

        public static bool operator ==(Quaternionf value1, Quaternionf value2)
        {
            return value1.X == value2.X && value1.Y == value2.Y && value1.Z == value2.Z && value1.W == value2.W;
        }

        public static bool operator !=(Quaternionf value1, Quaternionf value2)public void FromRotationMatrix(Matrix4f m)
        {
            float trace = m.Trace();

            if (trace > Single.Epsilon)
            {
                float s = (float)System.Math.Sqrt(trace + 1f);
                Vector.W = s * 0.5f;
                s = 0.5f / s;
                Vector.X = (m.M23 - m.M32) * s;
                Vector.Y = (m.M31 - m.M13) * s;
                Vector.Z = (m.M12 - m.M21) * s;
            }
            else
            {
                if (m.M11 > m.M22 && m.M11 > m.M33)
                {
                    float s = (float)System.Math.Sqrt(1f + m.M11 - m.M22 - m.M33);
                    Vector.X = 0.5f * s;
                    s = 0.5f / s;
                    Vector.Y = (m.M12 + m.M21) * s;
                    Vector.Z = (m.M13 + m.M31) * s;
                    Vector.W = (m.M23 - m.M32) * s;
                }
                else if (m.M22 > m.M33)
                {
                    float s = (float)System.Math.Sqrt(1f + m.M22 - m.M11 - m.M33);
                    Vector.Y = 0.5f * s;
                    s = 0.5f / s;
                    Vector.X = (m.M21 + m.M12) * s;
                    Vector.Z = (m.M32 + m.M23) * s;
                    Vector.W = (m.M31 - m.M13) * s;
                }
                else
                {
                    float s = (float)System.Math.Sqrt(1f + m.M33 - m.M11 - m.M22);
                    Vector.Z = 0.5f * s;
                    s = 0.5f / s;
                    Vector.X = (m.M31 + m.M13) * s;
                    Vector.Y = (m.M32 + m.M23) * s;
                    Vector.W = (m.M12 - m.M21) * s;
                }
            }
        }
        {
            return value1.X != value2.X || value1.Y != value2.Y || value1.Z != value2.Z || value1.W != value2.W;
        }

        public static Quaternionf operator +(Quaternionf value1, Quaternionf value2)
        {
            return new Quaternionf(
                value1.X + value2.X,
                value1.Y + value2.Y,
                value1.Z + value2.Z,
                value1.W + value2.W);
        }

        public static Quaternionf operator -(Quaternionf value)
        {
            return value.Negate();
        }

        public static Quaternionf operator -(Quaternionf value1, Quaternionf value2)
        {
            return new Quaternionf(
                value1.X - value2.X,
                value1.Y - value2.Y,
                value1.Z - value2.Z,
                value1.W - value2.W);
        }

        public static Quaternionf operator *(Quaternionf q1, Quaternionf q2)
        {
            return new Quaternionf(
                (q1.W * q2.X) + (q1.X * q2.W) + (q1.Y * q2.Z) - (q1.Z * q2.Y),
                (q1.W * q2.Y) - (q1.X * q2.Z) + (q1.Y * q2.W) + (q1.Z * q2.X),
                (q1.W * q2.Z) + (q1.X * q2.Y) - (q1.Y * q2.X) + (q1.Z * q2.W),
                (q1.W * q2.W) - (q1.X * q2.X) - (q1.Y * q2.Y) - (q1.Z * q2.Z));
        }

        public static Quaternionf operator *(Quaternionf value, float scaleFactor)
        {
            Vector4f scale = new Vector4f(scaleFactor);
            value.Vector *= scale;
            return value;
        }

        public static Quaternionf operator /(Quaternionf q1, Quaternionf q2)
        {
            float x = q1.X;
            float y = q1.Y;
            float z = q1.Z;
            float w = q1.W;

            float q2lensq = q2.LengthSquared(); //num14
            float ooq2lensq = 1f / q2lensq;
            float x2 = -q2.X * ooq2lensq;
            float y2 = -q2.Y * ooq2lensq;
            float z2 = -q2.Z * ooq2lensq;
            float w2 = q2.W * ooq2lensq;

            return new Quaternionf(
                ((x * w2) + (x2 * w)) + (y * z2) - (z * y2),
                ((y * w2) + (y2 * w)) + (z * x2) - (x * z2),
                ((z * w2) + (z2 * w)) + (x * y2) - (y * x2),
                (w * w2) - ((x * x2) + (y * y2)) + (z * z2));
        }

        #endregion Operators
    }
}
