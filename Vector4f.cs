// Vector4f.cs
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

using Mono.Simd;

namespace Mono.Simd.Math {
	
	public static class Vector4fExtensions
    {
        public static void Negate(this Vector4f vec)
        {
            vec *= Vector4f.MinusOne;
        }

        public static void Normalize(this Vector4f vec)
        {
            Vector4f factor = vec * vec;
            factor = factor.HorizontalAdd(factor);
            factor = factor.HorizontalAdd(factor);
            factor = factor.InvSqrt();
            vec *= factor;
        }

        public static float Length(this Vector4f vec)
        {
            Vector4f length = vec * vec;
            length = length.HorizontalAdd(length);
            length = length.HorizontalAdd(length);
            return length.Sqrt().X;
        }

        public static float LengthSquared(this Vector4f vec)
        {
            Vector4f length = vec * vec;
            length = length.HorizontalAdd(length);
            return length.HorizontalAdd(length).X;
        }

        public static bool ApproxEquals(this Vector4f vec, Vector4f vector, float tolerance)
        {
            Vector4f diff = vec - vector;
            return (diff.Length() <= tolerance);
        }

        public static bool IsFinite(this Vector4f vec)
        {
            return (Utils.IsFinite(vec.X) && Utils.IsFinite(vec.Y) && Utils.IsFinite(vec.Z) && Utils.IsFinite(vec.W));
        }
		
		
    }
}