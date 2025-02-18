// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import java.util.Arrays;
import java.util.Objects;
import java.util.StringJoiner;

/**
 * Taken from Bela Ban (https://github.com/belaban)
 * 
 * A hashmap where keys have to be ints. The size is fixed and keys/values are stored in a simple array, e.g.
 * [K1,V1, K2,V2, ... Kn,Vn]. The keys are indices into the array.<br/>
 * Typically populated at startup and then used in read-only mode. This implementation is unsynchronized, and keys
 * and values have to be non-null. Note that removals and changes (e.g. adding the same key twice) are unsupported.<br/>
 * Note that this class does not lend itself to be used as a sparse array, ie. adding keys [0 .. 100] and then 5000
 * would create an array of length 5001, and waste the space between index 100 and 5000. Ideally, keys start as 0, so
 * adding keys [50..100] would also waste the space for 50 keys.
 * @author Bela Ban
 */
public class IntHashMap<T> {
    protected T[] table;
    protected int size;

    /**
     * Creates an instance
     * @param highest_key The key with the highest value. The underlying array will be sized as highest_key+1
     */
    @SuppressWarnings("unchecked")
    public IntHashMap(int highest_key) {
        table = (T[])new Object[nonNegativeValue(highest_key) + 1];
    }

    public int           getCapacity()           {return table.length;}
    public int           size()                  {return size;}
    public boolean       isEmpty()               {return size == 0;}
    public boolean       containsKey(int k)      {return k+1 <= table.length && table[k] != null;}
    public T             get(int k)              {return k+1 <= table.length ?  table[k] :  null;}

    public IntHashMap<T> put(int key, T value) {
        checkCapacity(key);
        T val = table[key];
        if(val != null)
            throw new IllegalStateException(String.format("duplicate key %d (value: %s)", key, val));
        table[key] = Objects.requireNonNull(value);
        size++;
        return this;
    }

    public String toString() {
        StringJoiner sj = new StringJoiner(", ", "{", "}");
        for(int i = 0; i < table.length; i++) {
            T el = table[i];
            if(el != null)
                sj.add(i + "=" + el.toString());
        }
        return sj.toString();
    }

    protected IntHashMap<T> checkCapacity(int key) {
        if(key+1 > table.length) {
            int new_len=key+1;
            table=Arrays.copyOf(table, new_len);
        }
        return this;
    }

    public static <T extends Number> T nonNegativeValue(T num) {
        if(num.longValue() < 0)
            throw new IllegalArgumentException(String.format("%s must not be negative", num));
        return num;
    }
}
