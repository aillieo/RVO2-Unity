// -----------------------------------------------------------------------
// <copyright file="FixedPointTests.cs" company="AillieoTech">
// Copyright (c) AillieoTech. All rights reserved.
// </copyright>
// -----------------------------------------------------------------------

// =============================================================================
//  FixedPointTests.cs   EditMode 单元测试
//  放在 Assets/Tests/Editor/ 下,需在 asmdef 中引用 nunit + Deterministic
// =============================================================================
#if UNITY_INCLUDE_TESTS
using System.Globalization;
using NUnit.Framework;
using RVO;

/// <summary>
/// Unit tests for <see cref="FixedPoint"/>, <see cref="FixedPoint2"/>,
/// <see cref="FixedPointMath"/>, and <see cref="FixedPointRandom"/>.
/// </summary>
public class FixedPointTests
{
    private static readonly FixedPoint Half = FixedPoint.FromRaw(FixedPoint.OneRaw >> 1);
    private static readonly FixedPoint Quarter = FixedPoint.FromRaw(FixedPoint.OneRaw >> 2);

    // ---------- 基本运算 ----------

    /// <summary>Verifies basic add, subtract, multiply, and divide operations.</summary>
    [Test]
    public void Add_Sub_Mul_Div_Basic()
    {
        FixedPoint a = (FixedPoint)3;
        FixedPoint b = (FixedPoint)2;
        Assert.AreEqual((FixedPoint)5, a + b);
        Assert.AreEqual((FixedPoint)1, a - b);
        Assert.AreEqual((FixedPoint)6, a * b);
        // 3 / 2 = 1.5
        Assert.AreEqual(FixedPoint.FromRaw(FixedPoint.OneRaw + (FixedPoint.OneRaw >> 1)), a / b);
    }

    /// <summary>Verifies zero and sign behavior for basic arithmetic operators.</summary>
    [Test]
    public void Add_Sub_Negate_ZeroAndSigns()
    {
        var a = FixedPoint.FromRatio(3, 2);
        var b = FixedPoint.FromRatio(-5, 4);

        Assert.AreEqual(a, a + FixedPoint.Zero);
        Assert.AreEqual(FixedPoint.Zero, a - a);
        Assert.AreEqual(FixedPoint.FromRatio(1, 4), a + b);
        Assert.AreEqual(FixedPoint.FromRatio(11, 4), a - b);
        Assert.AreEqual(FixedPoint.FromRatio(-3, 2), -a);
        Assert.AreEqual(FixedPoint.Zero, -FixedPoint.Zero);
    }

    /// <summary>Verifies fractional multiplication preserves the high half of the 64-bit product.</summary>
    [Test]
    public void Mul_FractionalValues()
    {
        var almostOne = FixedPoint.FromRaw(FixedPoint.OneRaw - 1);

        Assert.AreEqual(Quarter, Half * Half);
        Assert.AreEqual(-Quarter, -Half * Half);
        Assert.AreEqual(FixedPoint.FromRaw((FixedPoint.OneRaw * 2) - 2), almostOne * (FixedPoint)2);
        Assert.AreEqual(FixedPoint.Zero, almostOne * FixedPoint.Zero);
        Assert.AreEqual(FixedPoint.FromRatio(-3, 8), FixedPoint.FromRatio(-3, 4) * Half);
        Assert.AreEqual(FixedPoint.FromRatio(3, 8), FixedPoint.FromRatio(-3, 4) * -Half);
    }

    /// <summary>Verifies division for fractional values and signs.</summary>
    [Test]
    public void Div_FractionalValuesAndSigns()
    {
        Assert.AreEqual(FixedPoint.FromRatio(3, 2), (FixedPoint)3 / (FixedPoint)2);
        Assert.AreEqual(FixedPoint.FromRatio(1, 4), Half / (FixedPoint)2);
        Assert.AreEqual((FixedPoint)2, Half / Quarter);
        Assert.AreEqual(FixedPoint.FromRatio(-3, 2), (FixedPoint)(-3) / (FixedPoint)2);
        Assert.AreEqual(FixedPoint.FromRatio(-3, 2), (FixedPoint)3 / (FixedPoint)(-2));
        Assert.AreEqual(FixedPoint.FromRatio(3, 2), (FixedPoint)(-3) / (FixedPoint)(-2));
        Assert.AreEqual(FixedPoint.Zero, FixedPoint.Zero / (FixedPoint)3);
    }

    /// <summary>Verifies modulo behavior and divide-by-zero exceptions.</summary>
    [Test]
    public void Mod_And_DivideByZero()
    {
        Assert.AreEqual((FixedPoint)1, (FixedPoint)7 % (FixedPoint)3);
        Assert.AreEqual((FixedPoint)(-1), (FixedPoint)(-7) % (FixedPoint)3);
        Assert.AreEqual(Quarter, FixedPoint.FromRatio(5, 4) % Half);
        Assert.Throws<System.DivideByZeroException>(() => { var _ = (FixedPoint)1 / FixedPoint.Zero; });
        Assert.Throws<System.DivideByZeroException>(() => { var _ = (FixedPoint)1 % FixedPoint.Zero; });
    }

    /// <summary>Verifies equality, comparison operators, and comparer methods.</summary>
    [Test]
    public void Comparison_OperatorsAndMethods()
    {
        var a = FixedPoint.FromRatio(3, 2);
        var b = FixedPoint.FromRatio(6, 4);
        var c = FixedPoint.FromRatio(7, 4);

        Assert.IsTrue(a == b);
        Assert.IsFalse(a != b);
        Assert.IsTrue(a < c);
        Assert.IsTrue(c > a);
        Assert.IsTrue(a <= b);
        Assert.IsTrue(a >= b);
        Assert.IsTrue(a.Equals(b));
        Assert.IsTrue(a.Equals((object)b));
        Assert.IsFalse(a.Equals("1.5"));
        Assert.AreEqual(0, a.CompareTo(b));
        Assert.Less(a.CompareTo(c), 0);
        Assert.AreEqual(a.GetHashCode(), b.GetHashCode());
    }

    /// <summary>Verifies casts to and from primitive numeric types.</summary>
    [Test]
    public void Casts_PrimitiveTypes()
    {
        // (-0.5) -> 0,(-1.5) -> -1,(0.5) -> 0,(1.5) -> 1
        long half = FixedPoint.OneRaw / 2;
        Assert.AreEqual(0,  (int)(FixedPoint.FromRaw(-half)));
        Assert.AreEqual(-1, (int)(FixedPoint.FromRaw(-(FixedPoint.OneRaw + half))));
        Assert.AreEqual(0,  (int)(FixedPoint.FromRaw(half)));
        Assert.AreEqual(1,  (int)(FixedPoint.FromRaw(FixedPoint.OneRaw + half)));
        Assert.AreEqual((short)-2, (short)FixedPoint.FromRatio(-9, 4));
        Assert.AreEqual(-2L, (long)FixedPoint.FromRatio(-9, 4));
        Assert.AreEqual((FixedPoint)123, (FixedPoint)(short)123);
        Assert.AreEqual((FixedPoint)123456, (FixedPoint)123456L);
        Assert.AreEqual(FixedPoint.FromRaw(FixedPoint.OneRaw + (FixedPoint.OneRaw >> 2)), (FixedPoint)1.25f);
        Assert.AreEqual(1.25f, (float)FixedPoint.FromRatio(5, 4), 0.000001f);
    }

    /// <summary>Verifies conversion overflow checks.</summary>
    [Test]
    public void Casts_Overflow_Throws()
    {
        // long 超过 int 范围才溢出 (Q31.32 整数部分容量 = int 范围)
        Assert.Throws<System.OverflowException>(() => { var _ = (FixedPoint)((long)int.MaxValue + 1L); });
        Assert.Throws<System.OverflowException>(() => { var _ = (FixedPoint)((long)int.MinValue - 1L); });
        Assert.Throws<System.OverflowException>(() => { var _ = (FixedPoint)float.PositiveInfinity; });
    }

    /// <summary>Verifies large value multiplication does not overflow.</summary>
    [Test]
    public void LargeRange_NoOverflow()
    {
        // Q31.32 应能容纳 ~21 亿
        FixedPoint a = (FixedPoint)1_000_000;
        FixedPoint b = (FixedPoint)2_000;
        Assert.AreEqual((FixedPoint)2_000_000_000, a * b);
    }

    /// <summary>Verifies raw construction and string formatting.</summary>
    [Test]
    public void Raw_And_StringFormatting()
    {
        var value = FixedPoint.FromRaw(FixedPoint.OneRaw + (FixedPoint.OneRaw >> 1));
        Assert.AreEqual(FixedPoint.OneRaw + (FixedPoint.OneRaw >> 1), value.RawValue);
        Assert.AreEqual("1.50000000", value.ToString());
        Assert.AreEqual("1.50", value.ToString("F2"));
        Assert.AreEqual("1.50", ((float)value).ToString("F2", CultureInfo.InvariantCulture));
    }

    // ---------- Sqrt ----------

    /// <summary>Verifies square root for perfect squares.</summary>
    [Test]
    public void Sqrt_PerfectSquares()
    {
        Assert.AreEqual((FixedPoint)2,    FixedPointMath.Sqrt((FixedPoint)4));
        Assert.AreEqual((FixedPoint)10,   FixedPointMath.Sqrt((FixedPoint)100));
        Assert.AreEqual((FixedPoint)1000, FixedPointMath.Sqrt((FixedPoint)1_000_000));
        Assert.AreEqual(FixedPoint.Zero,  FixedPointMath.Sqrt(FixedPoint.Zero));
        Assert.AreEqual(FixedPoint.One,   FixedPointMath.Sqrt(FixedPoint.One));
        Assert.AreEqual(Half, FixedPointMath.Sqrt(Quarter));
        Assert.AreEqual(FixedPoint.FromRatio(3, 2), FixedPointMath.Sqrt(FixedPoint.FromRatio(9, 4)));
    }

    /// <summary>Verifies square root for non-perfect squares is bounded correctly.</summary>
    [Test]
    public void Sqrt_NonPerfectSquares_Bounded()
    {
        var value = (FixedPoint)2;
        var sqrt = FixedPointMath.Sqrt(value);
        Assert.LessOrEqual((sqrt * sqrt).RawValue, value.RawValue);
        Assert.Greater(sqrt.RawValue, FixedPoint.FromRatio(1414, 1000).RawValue);
        Assert.Less(sqrt.RawValue, FixedPoint.FromRatio(1415, 1000).RawValue);
    }

    /// <summary>Verifies that square root of a negative number throws.</summary>
    [Test]
    public void Sqrt_Negative_Throws()
    {
        Assert.Throws<System.ArgumentException>(() => FixedPointMath.Sqrt((FixedPoint)(-1)));
    }

    // ---------- Math ----------

    /// <summary>Verifies min, max, clamp, abs, and sign helpers.</summary>
    [Test]
    public void Math_MinMaxClampAbsSign()
    {
        var low = (FixedPoint)(-2);
        var mid = Half;
        var high = (FixedPoint)3;

        Assert.AreEqual(low, FixedPointMath.Min(low, high));
        Assert.AreEqual(high, FixedPointMath.Max(low, high));
        Assert.AreEqual(low, FixedPointMath.Clamp((FixedPoint)(-5), low, high));
        Assert.AreEqual(high, FixedPointMath.Clamp((FixedPoint)5, low, high));
        Assert.AreEqual(mid, FixedPointMath.Clamp(mid, low, high));
        Assert.AreEqual(FixedPoint.Zero, FixedPointMath.Clamp01(low));
        Assert.AreEqual(FixedPoint.One, FixedPointMath.Clamp01(high));
        Assert.AreEqual(mid, FixedPointMath.Clamp01(mid));
        Assert.AreEqual((FixedPoint)2, FixedPointMath.Abs(low));
        Assert.AreEqual(0, FixedPointMath.Sign(FixedPoint.Zero));
        Assert.AreEqual(1, FixedPointMath.Sign(high));
        Assert.AreEqual(-1, FixedPointMath.Sign(low));
    }

    /// <summary>Verifies floor, ceil, and round for positive and negative fractions.</summary>
    [Test]
    public void Math_FloorCeilRound()
    {
        Assert.AreEqual((FixedPoint)1, FixedPointMath.Floor(FixedPoint.FromRatio(7, 4)));
        Assert.AreEqual((FixedPoint)2, FixedPointMath.Ceil(FixedPoint.FromRatio(7, 4)));
        Assert.AreEqual((FixedPoint)2, FixedPointMath.Round(FixedPoint.FromRatio(7, 4)));
        Assert.AreEqual((FixedPoint)1, FixedPointMath.Round(FixedPoint.FromRatio(5, 4)));
        Assert.AreEqual((FixedPoint)(-2), FixedPointMath.Floor(FixedPoint.FromRatio(-7, 4)));
        Assert.AreEqual((FixedPoint)(-1), FixedPointMath.Ceil(FixedPoint.FromRatio(-7, 4)));
        Assert.AreEqual((FixedPoint)(-2), FixedPointMath.Round(FixedPoint.FromRatio(-7, 4)));
        Assert.AreEqual((FixedPoint)(-1), FixedPointMath.Round(FixedPoint.FromRatio(-5, 4)));
        Assert.AreEqual((FixedPoint)2, FixedPointMath.Floor((FixedPoint)2));
        Assert.AreEqual((FixedPoint)2, FixedPointMath.Ceil((FixedPoint)2));
    }

    /// <summary>Verifies interpolation helpers including unclamped and clamped t values.</summary>
    [Test]
    public void Math_Lerp()
    {
        Assert.AreEqual((FixedPoint)5, FixedPointMath.Lerp((FixedPoint)0, (FixedPoint)10, Half));
        Assert.AreEqual((FixedPoint)(-5), FixedPointMath.Lerp((FixedPoint)0, (FixedPoint)10, -Half));
        Assert.AreEqual((FixedPoint)15, FixedPointMath.Lerp((FixedPoint)0, (FixedPoint)10, FixedPoint.FromRatio(3, 2)));
        Assert.AreEqual((FixedPoint)0, FixedPointMath.LerpClamped((FixedPoint)0, (FixedPoint)10, -Half));
        Assert.AreEqual((FixedPoint)10, FixedPointMath.LerpClamped((FixedPoint)0, (FixedPoint)10, FixedPoint.FromRatio(3, 2)));
        Assert.AreEqual((FixedPoint)5, FixedPointMath.LerpClamped((FixedPoint)0, (FixedPoint)10, Half));
    }

    // ---------- Vector2 ----------

    /// <summary>Verifies vector constants and component-wise operators.</summary>
    [Test]
    public void Vector_ConstantsAndOperators()
    {
        Assert.AreEqual(new FixedPoint2(FixedPoint.Zero, FixedPoint.Zero), FixedPoint2.Zero);
        Assert.AreEqual(new FixedPoint2(FixedPoint.One, FixedPoint.One), FixedPoint2.One);
        Assert.AreEqual(new FixedPoint2(FixedPoint.Zero, FixedPoint.One), FixedPoint2.Up);
        Assert.AreEqual(new FixedPoint2(FixedPoint.Zero, -FixedPoint.One), FixedPoint2.Down);
        Assert.AreEqual(new FixedPoint2(-FixedPoint.One, FixedPoint.Zero), FixedPoint2.Left);
        Assert.AreEqual(new FixedPoint2(FixedPoint.One, FixedPoint.Zero), FixedPoint2.Right);

        var a = new FixedPoint2((FixedPoint)3, (FixedPoint)(-4));
        var b = new FixedPoint2((FixedPoint)(-1), (FixedPoint)2);
        Assert.AreEqual(new FixedPoint2((FixedPoint)2, (FixedPoint)(-2)), a + b);
        Assert.AreEqual(new FixedPoint2((FixedPoint)4, (FixedPoint)(-6)), a - b);
        Assert.AreEqual(new FixedPoint2((FixedPoint)(-3), (FixedPoint)4), -a);
        Assert.AreEqual(new FixedPoint2((FixedPoint)(-3), (FixedPoint)(-8)), a * b);
        Assert.AreEqual(new FixedPoint2((FixedPoint)6, (FixedPoint)(-8)), a * (FixedPoint)2);
        Assert.AreEqual(new FixedPoint2((FixedPoint)6, (FixedPoint)(-8)), (FixedPoint)2 * a);
        Assert.AreEqual(new FixedPoint2(FixedPoint.FromRatio(3, 2), (FixedPoint)(-2)), a / (FixedPoint)2);
    }

    /// <summary>Verifies dot product and 2D cross product.</summary>
    [Test]
    public void Vector_DotCross()
    {
        var a = new FixedPoint2((FixedPoint)3, (FixedPoint)4);
        var b = new FixedPoint2((FixedPoint)1, (FixedPoint)2);
        Assert.AreEqual((FixedPoint)11, FixedPoint2.Dot(a, b));   // 3+8
        Assert.AreEqual((FixedPoint)2,  FixedPoint2.Cross(a, b)); // 6-4
        Assert.AreEqual(FixedPoint.Zero, FixedPoint2.Dot(FixedPoint2.Right, FixedPoint2.Up));
        Assert.AreEqual(FixedPoint.One, FixedPoint2.Cross(FixedPoint2.Right, FixedPoint2.Up));
    }

    /// <summary>Verifies distance and squared distance calculations.</summary>
    [Test]
    public void Vector_Distance()
    {
        var a = new FixedPoint2((FixedPoint)0, (FixedPoint)0);
        var b = new FixedPoint2((FixedPoint)3, (FixedPoint)4);
        Assert.AreEqual((FixedPoint)25, b.SqrMagnitude);
        Assert.AreEqual((FixedPoint)5, b.Magnitude);
        Assert.AreEqual((FixedPoint)5,  FixedPoint2.Distance(a, b));
        Assert.AreEqual((FixedPoint)25, FixedPoint2.SqrDistance(a, b));
        Assert.AreEqual(FixedPoint.Zero, FixedPoint2.Distance(a, a));
        Assert.AreEqual(FixedPoint.Zero, FixedPoint2.SqrDistance(a, a));
    }

    /// <summary>Verifies normalized vectors and vector equality members.</summary>
    [Test]
    public void Vector_Normalized_AndEquality()
    {
        var zero = FixedPoint2.Zero;
        var v = new FixedPoint2((FixedPoint)3, (FixedPoint)4);
        var n = v.Normalized;

        Assert.AreEqual(FixedPoint2.Zero, zero.Normalized);
        Assert.AreEqual(FixedPoint.FromRatio(3, 5), n.x);
        Assert.AreEqual(FixedPoint.FromRatio(4, 5), n.y);
        Assert.IsTrue(v == new FixedPoint2((FixedPoint)3, (FixedPoint)4));
        Assert.IsTrue(v != new FixedPoint2((FixedPoint)4, (FixedPoint)3));
        Assert.IsTrue(v.Equals(new FixedPoint2((FixedPoint)3, (FixedPoint)4)));
        Assert.IsTrue(v.Equals((object)new FixedPoint2((FixedPoint)3, (FixedPoint)4)));
        Assert.IsFalse(v.Equals("3,4"));
        Assert.AreEqual(v.GetHashCode(), new FixedPoint2((FixedPoint)3, (FixedPoint)4).GetHashCode());
        Assert.AreEqual("(3.00000000, 4.00000000)", v.ToString());
    }

    // ---------- Random 确定性 ----------

    /// <summary>Verifies that the same seed produces the same random sequence.</summary>
    [Test]
    public void Random_SameSeed_SameSequence()
    {
        var r1 = new FixedPointRandom(12345);
        var r2 = new FixedPointRandom(12345);
        for (int i = 0; i < 1000; i++)
            Assert.AreEqual(r1.NextUInt(), r2.NextUInt());
    }

    /// <summary>Verifies seed normalization and state round-tripping.</summary>
    [Test]
    public void Random_StateAndSeed()
    {
        var zeroSeed = new FixedPointRandom(0);
        Assert.AreEqual(1u, zeroSeed.State);

        zeroSeed.State = 0;
        Assert.AreEqual(1u, zeroSeed.State);

        var fromNegative = FixedPointRandom.FromSeed(-1);
        Assert.AreEqual(uint.MaxValue, fromNegative.State);

        var r = new FixedPointRandom(123);
        r.NextUInt();
        var state = r.State;
        var resumed = new FixedPointRandom(1) { State = state };
        Assert.AreEqual(r.NextUInt(), resumed.NextUInt());
    }

    /// <summary>Verifies integer random ranges and invalid range handling.</summary>
    [Test]
    public void Random_NextInt_Ranges()
    {
        var r = new FixedPointRandom(17);
        for (int i = 0; i < 1000; i++)
        {
            var v = r.NextInt();
            Assert.GreaterOrEqual(v, 0);
        }

        Assert.AreEqual(5, r.NextInt(5, 5));
        Assert.Throws<System.ArgumentException>(() => r.NextInt(6, 5));

        for (int i = 0; i < 1000; i++)
        {
            var v = r.NextInt(-3, 4);
            Assert.GreaterOrEqual(v, -3);
            Assert.Less(v, 4);
        }
    }

    /// <summary>Verifies that NextFixed returns values in [0, 1).</summary>
    [Test]
    public void Random_NextFixed_InRange()
    {
        var r = new FixedPointRandom(7);
        for (int i = 0; i < 10000; i++)
        {
            FixedPoint v = r.NextFixed();
            Assert.IsTrue(v >= FixedPoint.Zero && v < FixedPoint.One);
        }
    }

    /// <summary>Verifies ranged FixedPoint random values.</summary>
    [Test]
    public void Random_NextFixed_Range()
    {
        var r = new FixedPointRandom(23);
        var min = FixedPoint.FromRatio(-3, 2);
        var max = FixedPoint.FromRatio(7, 4);

        for (int i = 0; i < 1000; i++)
        {
            var v = r.NextFixed(min, max);
            Assert.GreaterOrEqual(v.RawValue, min.RawValue);
            Assert.Less(v.RawValue, max.RawValue);
        }
    }

    /// <summary>Verifies that InsideUnitCircle returns points inside the unit circle.</summary>
    [Test]
    public void Random_InsideUnitCircle()
    {
        var r = new FixedPointRandom(99);
        for (int i = 0; i < 1000; i++)
        {
            var p = r.InsideUnitCircle();
            Assert.IsTrue(p.SqrMagnitude <= FixedPoint.One);
        }
    }

    /// <summary>Verifies that FromRatio produces correct results for various inputs.</summary>
    [Test]
    public void ConstructFromRatio()
    {
        Assert.AreEqual((FixedPoint)1 / (FixedPoint)2, FixedPoint.FromRatio(1, 2));
        Assert.AreEqual((FixedPoint)1 / (FixedPoint)4, FixedPoint.FromRatio(1, 4));
        Assert.AreEqual((FixedPoint)3 / (FixedPoint)4, FixedPoint.FromRatio(3, 4));
        Assert.AreEqual((FixedPoint)0 / (FixedPoint)1, FixedPoint.FromRatio(0, 1));
        Assert.AreEqual((FixedPoint)1 / (FixedPoint)1, FixedPoint.FromRatio(1, 1));
        Assert.Throws<System.DivideByZeroException>(() => FixedPoint.FromRatio(1, 0));

        // 大数
        Assert.AreEqual((FixedPoint)1 / (FixedPoint)1000, FixedPoint.FromRatio(1, 1000));
        Assert.AreEqual((FixedPoint)123456 / (FixedPoint)654321, FixedPoint.FromRatio(123456, 654321));
        Assert.AreEqual((FixedPoint)123456789 / (FixedPoint)987654321, FixedPoint.FromRatio(123456789, 987654321));
        Assert.AreEqual((FixedPoint)1000 / (FixedPoint)1, FixedPoint.FromRatio(1000, 1));
        Assert.AreEqual((FixedPoint)987654321 / (FixedPoint)123456789, FixedPoint.FromRatio(987654321, 123456789));
        Assert.AreEqual((FixedPoint)int.MaxValue / (FixedPoint)1, FixedPoint.FromRatio(int.MaxValue, 1));

        // 负数
        Assert.AreEqual((FixedPoint)(-1) / (FixedPoint)2, FixedPoint.FromRatio(-1, 2));
        Assert.AreEqual((FixedPoint)1 / (FixedPoint)(-2), FixedPoint.FromRatio(1, -2));
        Assert.AreEqual((FixedPoint)(-1) / (FixedPoint)(-2), FixedPoint.FromRatio(-1, -2));
        Assert.AreEqual(FixedPoint.FromRaw(-FixedPoint.OneRaw / 3), FixedPoint.FromRatio(-1, 3));
    }
}
#endif
