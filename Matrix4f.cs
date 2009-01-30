// Matrix4f.cs
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

namespace Mono.Simd.Math {
    [Serializable]
    [StructLayout(LayoutKind.Explicit, Pack = 0, Size = 64)]
    public struct Matrix4f {
        [FieldOffset(0)]
        private Vector4f R0;
        [FieldOffset(16)]
        private Vector4f R1;
        [FieldOffset(32)]
        private Vector4f R2;
        [FieldOffset(48)]
        private Vector4f R3;

        #region Properties

        public float M11 { get { return R0.X; } set { R0.X = value; } }
        public float M12 { get { return R0.Y; } set { R0.Y = value; } }
        public float M13 { get { return R0.Z; } set { R0.Z = value; } }
        public float M14 { get { return R0.W; } set { R0.W = value; } }
        public float M21 { get { return R1.X; } set { R1.X = value; } }
        public float M22 { get { return R1.Y; } set { R1.Y = value; } }
        public float M23 { get { return R1.Z; } set { R1.Z = value; } }
        public float M24 { get { return R1.W; } set { R1.W = value; } }
        public float M31 { get { return R2.X; } set { R2.X = value; } }
        public float M32 { get { return R2.Y; } set { R2.Y = value; } }
        public float M33 { get { return R2.Z; } set { R2.Z = value; } }
        public float M34 { get { return R2.W; } set { R2.W = value; } }
        public float M41 { get { return R3.X; } set { R3.X = value; } }
        public float M42 { get { return R3.Y; } set { R3.Y = value; } }
        public float M43 { get { return R3.Z; } set { R3.Z = value; } }
        public float M44 { get { return R3.W; } set { R3.W = value; } }

        #endregion Properties

        #region Constructors

        public Matrix4f(
            float m11, float m12, float m13, float m14,
            float m21, float m22, float m23, float m24,
            float m31, float m32, float m33, float m34,
            float m41, float m42, float m43, float m44)
        {
            R0 = new Vector4f(m11, m12, m13, m14);
            R1 = new Vector4f(m21, m22, m23, m24);
            R2 = new Vector4f(m31, m32, m33, m34);
            R3 = new Vector4f(m41, m42, m43, m44);
        }

        public Matrix4f(Vector4f r0, Vector4f r1, Vector4f r2, Vector4f r3)
        {
            R0 = r0;
            R1 = r1;
            R2 = r2;
            R3 = r3;
        }

        public Matrix4f(Matrix4f m)
        {
            R0 = m.R0;
            R1 = m.R1;
            R2 = m.R2;
            R3 = m.R3;
        }

        #endregion Constructors

        #region Public Methods

        public float Determinant()
        {
            return
                R0.W * R1.Z * R2.Y * R3.X - R0.Z * R1.W * R2.Y * R3.X - R0.W * R1.Y * R2.Z * R3.X + R0.Y * R1.W * R2.Z * R3.X +
                R0.Z * R1.Y * R2.W * R3.X - R0.Y * R1.Z * R2.W * R3.X - R0.W * R1.Z * R2.X * R3.Y + R0.Z * R1.W * R2.X * R3.Y +
                R0.W * R1.X * R2.Z * R3.Y - R0.X * R1.W * R2.Z * R3.Y - R0.Z * R1.X * R2.W * R3.Y + R0.X * R1.Z * R2.W * R3.Y +
                R0.W * R1.Y * R2.X * R3.Z - R0.Y * R1.W * R2.X * R3.Z - R0.W * R1.X * R2.Y * R3.Z + R0.X * R1.W * R2.Y * R3.Z +
                R0.Y * R1.X * R2.W * R3.Z - R0.X * R1.Y * R2.W * R3.Z - R0.Z * R1.Y * R2.X * R3.W + R0.Y * R1.Z * R2.X * R3.W +
                R0.Z * R1.X * R2.Y * R3.W - R0.X * R1.Z * R2.Y * R3.W - R0.Y * R1.X * R2.Z * R3.W + R0.X * R1.Y * R2.Z * R3.W;
        }

        public float Trace()
        {
            return R0.X + R1.Y + R2.Z + R3.W;
        }

        public void GetEulerAngles(out float roll, out float pitch, out float yaw)
        {
            double angleX, angleY, angleZ;
            double cx, cy, cz; // cosines
            double sx, sz; // sines

            angleY = System.Math.Asin(Utils.Clamp(R0.Z, -1f, 1f));
            cy = System.Math.Cos(angleY);

            if (System.Math.Abs(cy) > 0.005f)
            {
                // No gimbal lock
                cx = R2.Z / cy;
                sx = (-R1.Z) / cy;

                angleX = (float)System.Math.Atan2(sx, cx);

                cz = R0.X / cy;
                sz = (-R0.Y) / cy;

                angleZ = (float)System.Math.Atan2(sz, cz);
            }
            else
            {
                // Gimbal lock
                angleX = 0;

                cz = R1.Y;
                sz = R1.X;

                angleZ = System.Math.Atan2(sz, cz);
            }

            // Return only positive angles in [0,360]
            if (angleX < 0) angleX += 360d;
            if (angleY < 0) angleY += 360d;
            if (angleZ < 0) angleZ += 360d;

            roll = (float)angleX;
            pitch = (float)angleY;
            yaw = (float)angleZ;
        }

        public Quaternionf GetQuaternion()
        {
            Quaternionf quat = new Quaternionf();
            float trace = Trace() + 1f;

            if (trace > Single.Epsilon)
            {
                float s = 0.5f / (float)System.Math.Sqrt(trace);

                quat.X = (R2.Y - R1.Z) * s;
                quat.Y = (R0.Z - R2.X) * s;
                quat.Z = (R1.X - R0.Y) * s;
                quat.W = 0.25f / s;
            }
            else
            {
                if (R0.X > R1.Y && R0.X > R2.Z)
                {
                    float s = 2.0f * (float)System.Math.Sqrt(1.0f + R0.X - R1.Y - R2.Z);

                    quat.X = 0.25f * s;
                    quat.Y = (R0.Y + R1.X) / s;
                    quat.Z = (R0.Z + R2.X) / s;
                    quat.W = (R1.Z - R2.Y) / s;
                }
                else if (R1.Y > R2.Z)
                {
                    float s = 2.0f * (float)System.Math.Sqrt(1.0f + R1.Y - R0.X - R2.Z);

                    quat.X = (R0.Y + R1.X) / s;
                    quat.Y = 0.25f * s;
                    quat.Z = (R1.Z + R2.Y) / s;
                    quat.W = (R0.Z - R2.X) / s;
                }
                else
                {
                    float s = 2.0f * (float)System.Math.Sqrt(1.0f + R2.Z - R0.X - R1.Y);

                    quat.X = (R0.Z + R2.X) / s;
                    quat.Y = (R1.Z + R2.Y) / s;
                    quat.Z = 0.25f * s;
                    quat.W = (R0.Y - R1.X) / s;
                }
            }

            return quat;
        }

        public void Negate()
        {
            R0 *= Vector4f.MinusOne;
            R1 *= Vector4f.MinusOne;
            R2 *= Vector4f.MinusOne;
            R3 *= Vector4f.MinusOne;
        }

        public Matrix4f Transform(ref Quaternionf rotation)
        {
            float x2 = rotation.X + rotation.X;
            float y2 = rotation.Y + rotation.Y;
            float z2 = rotation.Z + rotation.Z;

            float a = (1f - rotation.Y * y2) - rotation.Z * z2;
            float b = rotation.X * y2 - rotation.W * z2;
            float c = rotation.X * z2 + rotation.W * y2;
            float d = rotation.X * y2 + rotation.W * z2;
            float e = (1f - rotation.X * x2) - rotation.Z * z2;
            float f = rotation.Y * z2 - rotation.W * x2;
            float g = rotation.X * z2 - rotation.W * y2;
            float h = rotation.Y * z2 + rotation.W * x2;
            float i = (1f - rotation.X * x2) - rotation.Y * y2;

            return new Matrix4f(
                new Vector4f(
                    ((R0.X * a) + (R0.Y * b)) + (R0.Z * c),
                    ((R0.X * d) + (R0.Y * e)) + (R0.Z * f),
                    ((R0.X * g) + (R0.Y * h)) + (R0.Z * i),
                    R0.W),
                new Vector4f(
                    ((R1.X * a) + (R1.Y * b)) + (R1.Z * c),
                    ((R1.X * d) + (R1.Y * e)) + (R1.Z * f),
                    ((R1.X * g) + (R1.Y * h)) + (R1.Z * i),
                    R1.W),
                new Vector4f(
                    ((R2.X * a) + (R2.Y * b)) + (R2.Z * c),
                    ((R2.X * d) + (R2.Y * e)) + (R2.Z * f),
                    ((R2.X * g) + (R2.Y * h)) + (R2.Z * i),
                    R2.W),
                new Vector4f(
                    ((R3.X * a) + (R3.Y * b)) + (R3.Z * c),
                    ((R3.X * d) + (R3.Y * e)) + (R3.Z * f),
                    ((R3.X * g) + (R3.Y * h)) + (R3.Z * i),
                    R3.W));
        }

        public Matrix4f Transpose()
        {
            return new Matrix4f(
                new Vector4f(R0.X, R1.X, R2.X, R3.X),
                new Vector4f(R0.Y, R1.Y, R2.Y, R3.Y),
                new Vector4f(R0.Z, R1.Z, R2.Z, R3.Z),
                new Vector4f(R0.W, R1.W, R2.W, R3.W));
        }

        public void FromAxisAngle(Vector3f axis, float angle)
        {
            float x = axis.X;
            float y = axis.Y;
            float z = axis.Z;
            float sin = (float)System.Math.Sin(angle);
            float cos = (float)System.Math.Cos(angle);
            float xx = x * x;
            float yy = y * y;
            float zz = z * z;
            float xy = x * y;
            float xz = x * z;
            float yz = y * z;

            R0.X = xx + (cos * (1f - xx));
            R0.Y = (xy - (cos * xy)) + (sin * z);
            R0.Z = (xz - (cos * xz)) - (sin * y);
            R0.W = 0f;

            R1.X = (xy - (cos * xy)) - (sin * z);
            R1.Y = yy + (cos * (1f - yy));
            R1.Z = (yz - (cos * yz)) + (sin * x);
            R1.W = 0f;

            R2.X = (xz - (cos * xz)) + (sin * y);
            R2.Y = (yz - (cos * yz)) - (sin * x);
            R2.Z = zz + (cos * (1f - zz));
            R2.W = 0f;

            R3.X = R3.Y = R3.Z = 0f;
            R3.W = 1f;
        }

        public void FromEulers(float roll, float pitch, float yaw)
        {
            float a, b, c, d, e, f;
            float ad, bd;

            a = (float)System.Math.Cos(roll);
            b = (float)System.Math.Sin(roll);
            c = (float)System.Math.Cos(pitch);
            d = (float)System.Math.Sin(pitch);
            e = (float)System.Math.Cos(yaw);
            f = (float)System.Math.Sin(yaw);

            ad = a * d;
            bd = b * d;

            R0.X = c * e;
            R0.Y = -c * f;
            R0.Z = d;
            R0.W = 0f;

            R1.X = bd * e + a * f;
            R1.Y = -bd * f + a * e;
            R1.Z = -b * c;
            R1.W = 0f;

            R2.X = -ad * e + b * f;
            R2.Y = ad * f + b * e;
            R2.Z = a * c;
            R2.W = 0f;

            R3.X = R3.Y = R3.Z = 0f;
            R3.W = 1f;
        }

        public void FromQuaternion(Quaternionf quaternion)
        {
            Matrix4f m1 = new Matrix4f(
                quaternion.W, quaternion.Z, -quaternion.Y, quaternion.X,
                -quaternion.Z, quaternion.W, quaternion.X, quaternion.Y,
                quaternion.Y, -quaternion.X, quaternion.W, quaternion.Z,
                -quaternion.X, -quaternion.Y, -quaternion.Z, quaternion.W);

            Matrix4f m2 = new Matrix4f(
                quaternion.W, quaternion.Z, -quaternion.Y, -quaternion.X,
                -quaternion.Z, quaternion.W, quaternion.X, -quaternion.Y,
                quaternion.Y, -quaternion.X, quaternion.W, -quaternion.Z,
                quaternion.X, quaternion.Y, quaternion.Z, quaternion.W);

            this = m1 * m2;
        }

        public void FromRotationX(float radians)
        {
            float cos = (float)System.Math.Cos(radians);
            float sin = (float)System.Math.Sin(radians);

            R0 = new Vector4f(1f, 0f, 0f, 0f);
            R1 = new Vector4f(0f, cos, sin, 0f);
            R2 = new Vector4f(0f, -sin, cos, 0f);
            R3 = new Vector4f(0f, 0f, 0f, 1f);
        }

        public void FromRotationY(float radians)
        {
            float cos = (float)System.Math.Cos(radians);
            float sin = (float)System.Math.Sin(radians);

            R0 = new Vector4f(cos, 0f, -sin, 0f);
            R1 = new Vector4f(0f, 1f, 0f, 0f);
            R2 = new Vector4f(sin, 0f, cos, 0f);
            R3 = new Vector4f(0f, 0f, 0f, 1f);
        }

        public void FromRotationZ(float radians)
        {
            float cos = (float)System.Math.Cos(radians);
            float sin = (float)System.Math.Sin(radians);

            R0 = new Vector4f(cos, sin, 0f, 0f);
            R1 = new Vector4f(-sin, cos, 0f, 0f);
            R2 = new Vector4f(0f, 0f, 1f, 0f);
            R3 = new Vector4f(0f, 0f, 0f, 1f);
        }

        public void FromScale(Vector3f scale)
        {
            R0 = new Vector4f(scale.X, 0f, 0f, 0f);
            R1 = new Vector4f(0f, scale.Y, 0f, 0f);
            R2 = new Vector4f(0f, 0f, scale.Z, 0f);
            R3 = new Vector4f(0f, 0f, 0f, 1f);
        }

        public void FromTranslation(Vector3f position)
        {
            R0 = new Vector4f(1f, 0f, 0f, 0f);
            R1 = new Vector4f(0f, 1f, 0f, 0f);
            R2 = new Vector4f(0f, 0f, 1f, 0f);
            R3 = new Vector4f(position.X, position.Y, position.Z, 1f);
        }

        public void FromWorld(Vector3f position, Vector3f forward, Vector3f up)
        {
            // Normalize forward vector
            forward.Normalize();

            // Calculate right vector
            Vector3f right = forward.Cross(ref up);
            right.Normalize();

            // Recalculate up vector
            up = right.Cross(ref forward);
            up.Normalize();

            R0 = new Vector4f(right.X, right.Y, right.Z, 0.0f);
            R1 = new Vector4f(up.X, up.Y, up.Z, 0.0f);
            R2 = new Vector4f(-forward.X, -forward.Y, -forward.Z, 0.0f);
            R3 = new Vector4f(position.X, position.Y, position.Z, 1.0f);
        }

        #endregion Public Methods

        #region Overrides

        public override int GetHashCode()
        {
            return
                R0.X.GetHashCode() ^ R0.Y.GetHashCode() ^ R0.Z.GetHashCode() ^ R0.W.GetHashCode() ^
                R1.X.GetHashCode() ^ R1.Y.GetHashCode() ^ R1.Z.GetHashCode() ^ R1.W.GetHashCode() ^
                R2.X.GetHashCode() ^ R2.Y.GetHashCode() ^ R2.Z.GetHashCode() ^ R2.W.GetHashCode() ^
                R3.X.GetHashCode() ^ R3.Y.GetHashCode() ^ R3.Z.GetHashCode() ^ R3.W.GetHashCode();
        }

        /// <summary>
        /// Get a formatted string representation of the vector
        /// </summary>
        /// <returns>A string representation of the vector</returns>
        public override string ToString()
        {
            return string.Format(
                "|{0}, {1}, {2}, {3}|\n" +
                "|{4}, {5}, {6}, {7}|\n" +
                "|{8}, {9}, {10}, {11}|\n" +
                "|{12}, {13}, {14}, {15}|",
                R0.X, R0.Y, R0.Z, R0.W,
                R1.X, R1.Y, R1.Z, R1.W,
                R2.X, R2.Y, R2.Z, R2.W,
                R3.X, R3.Y, R3.Z, R3.W);
        }

        #endregion Overrides

        #region Operators

        public static bool operator ==(Matrix4f left, Matrix4f right)
        {
            Vector4f result =
                left.R0.CompareNotEqual(right.R0) +
                left.R1.CompareNotEqual(right.R1) +
                left.R2.CompareNotEqual(right.R2) +
                left.R3.CompareNotEqual(right.R3);

            return result.X != 0f || result.Y != 0f || result.Y != 0f || result.Z != 0f;
        }

        public static bool operator !=(Matrix4f left, Matrix4f right)
        {
            Vector4f result =
                left.R0.CompareEqual(right.R0) +
                left.R1.CompareEqual(right.R1) +
                left.R2.CompareEqual(right.R2) +
                left.R3.CompareEqual(right.R3);

            return result.X == 0f || result.Y == 0f || result.Z == 0f || result.W == 0f;
        }

        public static Matrix4f operator +(Matrix4f left, Matrix4f right)
        {
            return new Matrix4f(
                left.R0 + right.R0,
                left.R1 + right.R1,
                left.R2 + right.R2,
                left.R3 + right.R3);
        }

        public static Matrix4f operator -(Matrix4f matrix)
        {
            matrix.Negate();
            return matrix;
        }

        public static Matrix4f operator -(Matrix4f left, Matrix4f right)
        {
            return new Matrix4f(
                left.R0 - right.R0,
                left.R1 - right.R1,
                left.R2 - right.R2,
                left.R3 - right.R3);
        }

        public static Matrix4f operator *(Matrix4f left, Matrix4f right)
        {
            Vector4f t1, t2;
            Vector4f out0, out1, out2, out3;

            t1 = (left.R0.Shuffle(ShuffleSel.ExpandX) * right.R0) + (left.R0.Shuffle(ShuffleSel.ExpandY) * right.R1);
            t2 = (left.R0.Shuffle(ShuffleSel.ExpandZ) * right.R2) + (left.R0.Shuffle(ShuffleSel.ExpandW) * right.R3);
            out0 = t1 + t2;

            t1 = (left.R1.Shuffle(ShuffleSel.ExpandX) * right.R0) + (left.R1.Shuffle(ShuffleSel.ExpandY) * right.R1);
            t2 = (left.R1.Shuffle(ShuffleSel.ExpandZ) * right.R2) + (left.R1.Shuffle(ShuffleSel.ExpandW) * right.R3);
            out1 = t1 + t2;

            t1 = (left.R2.Shuffle(ShuffleSel.ExpandX) * right.R0) + (left.R2.Shuffle(ShuffleSel.ExpandY) * right.R1);
            t2 = (left.R2.Shuffle(ShuffleSel.ExpandZ) * right.R2) + (left.R2.Shuffle(ShuffleSel.ExpandW) * right.R3);
            out2 = t1 + t2;

            t1 = (left.R3.Shuffle(ShuffleSel.ExpandX) * right.R0) + (left.R3.Shuffle(ShuffleSel.ExpandY) * right.R1);
            t2 = (left.R3.Shuffle(ShuffleSel.ExpandZ) * right.R2) + (left.R3.Shuffle(ShuffleSel.ExpandW) * right.R3);
            out3 = t1 + t2;

            return new Matrix4f(out0, out1, out2, out3);
        }

        public static Matrix4f operator *(Matrix4f left, float scaleFactor)
        {
            Vector4f v = new Vector4f(scaleFactor);

            return new Matrix4f(
                left.R0 * v,
                left.R1 * v,
                left.R2 * v,
                left.R3 * v);
        }

        public static Matrix4f operator /(Matrix4f left, Matrix4f right)
        {
            return new Matrix4f(
                left.R0 / right.R0,
                left.R1 / right.R1,
                left.R2 / right.R2,
                left.R3 / right.R3);
        }

        public static Matrix4f operator /(Matrix4f matrix, float divider)
        {
            float oodivider = 1f / divider;
            Vector4f v = new Vector4f(oodivider);

            return new Matrix4f(
                matrix.R0 * v,
                matrix.R1 * v,
                matrix.R2 * v,
                matrix.R3 * v);
        }

        [System.Runtime.CompilerServices.IndexerName("Component")]
        public Vector4f this[int row]
        {
            get
            {
                switch (row)
                {
                    case 0:
                        return R0;
                    case 1:
                        return R1;
                    case 2:
                        return R2;
                    case 3:
                        return R3;
                    default:
                        throw new IndexOutOfRangeException("row");
                }
            }
            set
            {
                switch (row)
                {
                    case 0:
                        R0 = value;
                        break;
                    case 1:
                        R1 = value;
                        break;
                    case 2:
                        R2 = value;
                        break;
                    case 3:
                        R3 = value;
                        break;
                    default:
                        throw new IndexOutOfRangeException("row");
                }
            }
        }

        [System.Runtime.CompilerServices.IndexerName("Component")]
        public float this[int row, int column]
        {
            get
            {
                switch (row)
                {
                    case 0:
                        switch (column)
                        {
                            case 0:
                                return R0.X;
                            case 1:
                                return R0.Y;
                            case 2:
                                return R0.Z;
                            case 3:
                                return R0.W;
                            default:
                                throw new IndexOutOfRangeException("column");
                        }
                    case 1:
                        switch (column)
                        {
                            case 0:
                                return R1.X;
                            case 1:
                                return R1.Y;
                            case 2:
                                return R1.Z;
                            case 3:
                                return R1.W;
                            default:
                                throw new IndexOutOfRangeException("column");
                        }
                    case 2:
                        switch (column)
                        {
                            case 0:
                                return R2.X;
                            case 1:
                                return R2.Y;
                            case 2:
                                return R2.Z;
                            case 3:
                                return R2.W;
                            default:
                                throw new IndexOutOfRangeException("column");
                        }
                    case 3:
                        switch (column)
                        {
                            case 0:
                                return R3.X;
                            case 1:
                                return R3.Y;
                            case 2:
                                return R3.Z;
                            case 3:
                                return R3.W;
                            default:
                                throw new IndexOutOfRangeException("column");
                        }
                    default:
                        throw new IndexOutOfRangeException("row");
                }
            }
        }

        #endregion Operators

        public static readonly Matrix4f Zero = new Matrix4f();

        public static readonly Matrix4f Identity = new Matrix4f(
            1f, 0f, 0f, 0f,
            0f, 1f, 0f, 0f,
            0f, 0f, 1f, 0f,
            0f, 0f, 0f, 1f);
    }
}
