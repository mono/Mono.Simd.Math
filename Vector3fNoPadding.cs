// Vector3fNoSimd.cs
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
    [StructLayout(LayoutKind.Explicit, Pack = 0, Size = 12)]
    public struct Vector3fNoPadding
    {
        [FieldOffset(0)]
        public float X;
        [FieldOffset(4)]
        public float Y;
        [FieldOffset(8)]
        public float Z;

        #region Properties

        #endregion Properties

        #region Constructors

        public Vector3fNoPadding(float x, float y, float z)
        {
            X = x;
            Y = y;
            Z = z;
        }

        public Vector3fNoPadding(float value)
        {
            X = value;
            Y = value;
            Z = value;
        }

        public Vector3fNoPadding(Vector3f vector)
        {
            X = vector.X;
            Y = vector.Y;
            Z = vector.Z;
        }

        #endregion Constructors

        #region Public Methods

        public void Negate()
        {
            X = -X;
            Y = -Y;
            Z = -Z;
        }

        public void Normalize()
        {
            const float MAG_THRESHOLD = 0.0000001f;
            float factor = (float)System.Math.Sqrt(X * X + Y * Y + Z * Z);
            if (factor > MAG_THRESHOLD)
            {
                factor = 1f / factor;
                X *= factor;
                Y *= factor;
                Z *= factor;
            }
            else
            {
                X = 0f;
                Y = 0f;
                Z = 0f;
            }
        }

        public float Length()
        {
            return (float)System.Math.Sqrt(X * X + Y * Y + Z * Z);
        }

        public float LengthSquared()
        {
            return X * X + Y * Y + Z * Z;
        }

        public bool ApproxEquals(Vector3f vec, float tolerance)
        {
            Vector3f diff = this - vec;
            return (diff.Length() <= tolerance);
        }

        public bool IsFinite()
        {
            return (Utils.IsFinite(X) && Utils.IsFinite(Y) && Utils.IsFinite(Z));
        }

        public Vector3f Clamp(ref Vector3f min, ref Vector3f max)
        {
            return new Vector3f(
                Utils.Clamp(X, min.X, max.X),
                Utils.Clamp(Y, min.Y, max.Y),
                Utils.Clamp(Z, min.Z, max.Z));
        }

        public Vector3f Cross(ref Vector3f vector)
        {
            return new Vector3f(
                Y * vector.Z - vector.Y * Z,
                Z * vector.X - vector.Z * X,
                X * vector.Y - vector.X * Y);
        }

        public float Distance(ref Vector3f vector)
        {
            return (float)System.Math.Sqrt(DistanceSquared(ref vector));
        }

        public float DistanceSquared(ref Vector3f vector)
        {
            return
                (X - vector.X) * (X - vector.X) +
                (Y - vector.Y) * (Y - vector.Y) +
                (Z - vector.Z) * (Z - vector.Z);
        }

        public float Dot(ref Vector3f vector)
        {
            return X * vector.X + Y * vector.Y + Z * vector.Z;
        }

        public Vector3f Lerp(ref Vector3f vector, float amount)
        {
            return new Vector3f(
                Utils.Lerp(X, vector.X, amount),
                Utils.Lerp(Y, vector.Y, amount),
                Utils.Lerp(Z, vector.Z, amount));
        }

        public Vector3f Max(ref Vector3f vector)
        {
            return new Vector3f(
                System.Math.Max(X, vector.X),
                System.Math.Max(Y, vector.Y),
                System.Math.Max(Z, vector.Z));
        }

        public Vector3f Min(ref Vector3f vector)
        {
            return new Vector3f(
                System.Math.Min(X, vector.X),
                System.Math.Min(Y, vector.Y),
                System.Math.Min(Z, vector.Z));
        }

        /// <summary>
        /// Calculate the rotation between this normalized directional vector
        /// and a normalized target vector
        /// </summary>
        /// <param name="target">Target vector</param>
        public Quaternionf RotationBetween(ref Vector3f target)
        {
            float dotProduct = Dot(ref target);
            Vector3f crossProduct = Cross(ref target);
            float magProduct = Length() * target.Length();
            double angle = System.Math.Acos(dotProduct / magProduct);
            crossProduct.Normalize();
            float s = (float)System.Math.Sin(angle / 2d);

            return new Quaternionf(
                crossProduct.X * s,
                crossProduct.Y * s,
                crossProduct.Z * s,
                (float)System.Math.Cos(angle / 2d));
        }

        /// <summary>
        /// Interpolates between this vector and another using a cubic equation
        /// </summary>
        /// <param name="target">Target vector</param>
        /// <param name="amount">Normalized amount to interpolate</param>
        /// <returns>A vector in between this vector and the target</returns>
        public Vector3f SmoothStep(ref Vector3f target, float amount)
        {
            return new Vector3f(
                Utils.SmoothStep(X, target.X, amount),
                Utils.SmoothStep(Y, target.Y, amount),
                Utils.SmoothStep(Z, target.Z, amount));
        }

        public Vector3f Transform(ref Matrix4f matrix)
        {
            return new Vector3f(
                (X * matrix.M11) + (Y * matrix.M21) + (Z * matrix.M31) + matrix.M41,
                (X * matrix.M12) + (Y * matrix.M22) + (Z * matrix.M32) + matrix.M42,
                (X * matrix.M13) + (Y * matrix.M23) + (Z * matrix.M33) + matrix.M43);
        }

        public Vector3f TransformNormal(ref Matrix4f matrix)
        {
            return new Vector3f(
                (X * matrix.M11) + (Y * matrix.M21) + (Z * matrix.M31),
                (X * matrix.M12) + (Y * matrix.M22) + (Z * matrix.M32),
                (X * matrix.M13) + (Y * matrix.M23) + (Z * matrix.M33));
        }

        #endregion Public Methods

        #region Overrides

        public override int GetHashCode()
        {
            return X.GetHashCode() ^ Y.GetHashCode() ^ Z.GetHashCode();
        }

        public override string ToString()
        {
            return "<" + X + ", " + Y + ", " + Z + ">"; 
        }

        [System.Runtime.CompilerServices.IndexerName ("Component")]
        public float this[int index]
        {
            get
            {
                switch (index)
                {
                    case 0:
                        return X;
                    case 1:
                        return Y;
                    case 2:
                        return Z;
                    default:
                        throw new IndexOutOfRangeException("index");
                }
            }
            set
            {
                switch (index)
                {
                    case 0:
                        X = value;
                        break;
                    case 1:
                        Y = value;
                        break;
                    case 2:
                        Z = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("index");
                }
            }
        }

        #endregion Overrides

        #region Operators

        public static bool operator ==(Vector3f value1, Vector3f value2)
        {
            return value1.X == value2.X && value1.Y == value2.Y && value1.Z == value2.Z;
        }

        public static bool operator !=(Vector3f value1, Vector3f value2)
        {
            return value1.X != value2.X || value1.Y != value2.Y || value1.Z != value2.Z;
        }

        public static Vector3f operator +(Vector3f value1, Vector3f value2)
        {
            return new Vector3f(
                value1.X + value2.X,
                value1.Y + value2.Y,
                value1.Z + value2.Z);
        }

        public static Vector3f operator -(Vector3f value)
        {
            return value * -1f;
        }

        public static Vector3f operator -(Vector3f value1, Vector3f value2)
        {
            return new Vector3f(
                value1.X - value2.X,
                value1.Y - value2.Y,
                value1.Z - value2.Z);
        }

        public static Vector3f operator *(Vector3f value1, Vector3f value2)
        {
            return new Vector3f(
                value1.X * value2.X,
                value1.Y * value2.Y,
                value1.Z * value2.Z);
        }

        public static Vector3f operator *(Vector3f value, float scaleFactor)
        {
            return new Vector3f(
                value.X * scaleFactor,
                value.Y * scaleFactor,
                value.Z * scaleFactor);
        }

        public static Vector3f operator *(Vector3f vec, Quaternionf rot)
        {
            Vector3f result;

            result.X =
                     rot.W * rot.W * vec.X +
                2f * rot.Y * rot.W * vec.Z -
                2f * rot.Z * rot.W * vec.Y +
                     rot.X * rot.X * vec.X +
                2f * rot.Y * rot.X * vec.Y +
                2f * rot.Z * rot.X * vec.Z -
                     rot.Z * rot.Z * vec.X -
                     rot.Y * rot.Y * vec.X;

            result.Y =
                2f * rot.X * rot.Y * vec.X +
                     rot.Y * rot.Y * vec.Y +
                2f * rot.Z * rot.Y * vec.Z +
                2f * rot.W * rot.Z * vec.X -
                     rot.Z * rot.Z * vec.Y +
                     rot.W * rot.W * vec.Y -
                2f * rot.X * rot.W * vec.Z -
                     rot.X * rot.X * vec.Y;

            result.Z =
                2f * rot.X * rot.Z * vec.X +
                2f * rot.Y * rot.Z * vec.Y +
                     rot.Z * rot.Z * vec.Z -
                2f * rot.W * rot.Y * vec.X -
                     rot.Y * rot.Y * vec.Z +
                2f * rot.W * rot.X * vec.Y -
                     rot.X * rot.X * vec.Z +
                     rot.W * rot.W * vec.Z;

            return result;
        }

        public static Vector3f operator *(Vector3f vector, Matrix4f matrix)
        {
            return vector.Transform(ref matrix);
        }

        public static Vector3f operator /(Vector3f value1, Vector3f value2)
        {
            return new Vector3f(
                value1.X / value2.X,
                value1.Y / value2.Y,
                value1.Z / value2.Z);
        }

        public static Vector3f operator /(Vector3f value, float divider)
        {
            float oodivider = 1f / divider;
            return new Vector3f(
                value.X * oodivider,
                value.Y * oodivider,
                value.Z * oodivider);
        }

        #endregion Operators

        /// <summary>A vector with a value of 0,0,0</summary>
        public readonly static Vector3f Zero = new Vector3f();
        /// <summary>A vector with a value of 1,1,1</summary>
        public readonly static Vector3f One = new Vector3f(1f, 1f, 1f);
        /// <summary>A vector with a value of -1,-1,-1</summary>
        public readonly static Vector3f MinusOne = new Vector3f(-1f, -1f, -1f);
        /// <summary>A unit vector facing forward (X axis), value 1,0,0</summary>
        public readonly static Vector3f UnitX = new Vector3f(1f, 0f, 0f);
        /// <summary>A unit vector facing left (Y axis), value 0,1,0</summary>
        public readonly static Vector3f UnitY = new Vector3f(0f, 1f, 0f);
        /// <summary>A unit vector facing up (Z axis), value 0,0,1</summary>
        public readonly static Vector3f UnitZ = new Vector3f(0f, 0f, 1f);
    }
}
