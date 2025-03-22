// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Objects;

/**
 * Represents a triad of three objects.
 *
 * @param <A> The first object's type.
 * @param <B> The second object's type.
 * @param <C> The third object's type.
 */
public class Triad<A, B, C> {
    private final A m_first;
    private final B m_second;
    private final C m_third;

    /**
     * Constructs a triad.
     *
     * @param first The first object.
     * @param second The second object.
     * @param third The third object.
     */
    public Triad(A first, B second, C third) {
        m_first = first;
        m_second = second;
        m_third = third;
    }

    /**
     * Returns the first object.
     *
     * @return The first object.
     */
    public A getFirst() {
        return m_first;
    }

    /**
     * Returns the second object.
     *
     * @return The second object.
     */
    public B getSecond() {
        return m_second;
    }

    /**
     * Returns the third object.
     *
     * @return The third object.
     */
    public C getThird() {
        return m_third;
    }

    /**
     * Returns a triad comprised of the three given objects.
     *
     * @param <A> The first object's type.
     * @param <B> The second object's type.
     * @param <C> The third object's type.
     * @param a The first object.
     * @param b The second object.
     * @param c the third object.
     * @return A pair comprised of the two given objects.
     */
    public static <A, B, C> Triad<A, B, C> of(A a, B b, C c) {
        return new Triad<A,B,C>(a, b, c);
    }

    @Override
    public String toString() {
        return String.format("Triad(%s, %s, %s)", m_first, m_second, m_third);
    }

    /**
     * Checks equality between this Pair and another object.
     *
     * @param obj The other object.
     * @return Whether the two objects are equal or not.
     */
    @Override
    public boolean equals(Object obj) {
        return obj == this
            || obj instanceof Triad<?, ?, ?> other
                && Objects.equals(m_first, other.getFirst())
                && Objects.equals(m_second, other.getSecond())
                && Objects.equals(m_third, other.getThird());
    }

    @Override
    public int hashCode() {
        return Objects.hash(m_first, m_second, m_third);
    }
}
