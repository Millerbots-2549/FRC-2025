// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.telemetry.reflect;

import java.io.File;
import java.io.IOException;
import java.lang.annotation.Annotation;
import java.lang.reflect.AccessibleObject;
import java.lang.reflect.Field;
import java.lang.reflect.Method;
import java.lang.reflect.Modifier;
import java.net.URL;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.IdentityHashMap;
import java.util.List;
import java.util.Map;
import java.util.Objects;
import java.util.Set;
import java.util.function.BiConsumer;
import java.util.function.Predicate;
import java.util.stream.Stream;

import frc.robot.util.IntHashMap;

/** Add your docs here. */
public class AnnotationProcessor {
    private static final Map<Class<?>,Byte>   TYPES=new IdentityHashMap<>(30);
    private static final IntHashMap<Class<?>> CLASS_TYPES=new IntHashMap<>(20);

    //#region types
    private static final byte    TYPE_NULL         =  0;
    private static final byte    TYPE_BOOLEAN      =  1; // boolean
    private static final byte    TYPE_BOOLEAN_OBJ  =  2; // Boolean
    private static final byte    TYPE_BYTE         =  3; // byte
    private static final byte    TYPE_BYTE_OBJ     =  4; // Byte
    private static final byte    TYPE_CHAR         =  5; // char
    private static final byte    TYPE_CHAR_OBJ     =  6; // Character
    private static final byte    TYPE_DOUBLE       =  7; // double
    private static final byte    TYPE_DOUBLE_OBJ   =  8; // Double
    private static final byte    TYPE_FLOAT        =  9; // float
    private static final byte    TYPE_FLOAT_OBJ    = 10; // Float
    private static final byte    TYPE_INT          = 11; // int
    private static final byte    TYPE_INT_OBJ      = 12; // Integer
    private static final byte    TYPE_LONG         = 13; // long
    private static final byte    TYPE_LONG_OBJ     = 14; // Long
    private static final byte    TYPE_SHORT        = 15; // short
    private static final byte    TYPE_SHORT_OBJ    = 16; // Short
    private static final byte    TYPE_STRING       = 17; // ascii
    private static final byte    TYPE_BYTEARRAY    = 18;
    private static final byte    TYPE_CLASS        = 19; // a class

    protected static void add(byte type, Class<?> cl) {
        if(TYPES.putIfAbsent(cl, type) != null)
            throw new IllegalStateException(String.format("type %d (class=%s) is already present in types map", type, cl));
        CLASS_TYPES.put(type, cl);
    }

    static {
        add(TYPE_NULL,         Void.class);
        add(TYPE_BOOLEAN,      boolean.class);
        add(TYPE_BOOLEAN_OBJ,  Boolean.class);
        add(TYPE_BYTE,         byte.class);
        add(TYPE_BYTE_OBJ,     Byte.class);
        add(TYPE_CHAR,         char.class);
        add(TYPE_CHAR_OBJ,     Character.class);
        add(TYPE_DOUBLE,       double.class);
        add(TYPE_DOUBLE_OBJ,   Double.class);
        add(TYPE_FLOAT,        float.class);
        add(TYPE_FLOAT_OBJ,    Float.class);
        add(TYPE_INT,          int.class);
        add(TYPE_INT_OBJ,      Integer.class);
        add(TYPE_LONG,         long.class);
        add(TYPE_LONG_OBJ,     Long.class);
        add(TYPE_SHORT,        short.class);
        add(TYPE_SHORT_OBJ,    Short.class);
        add(TYPE_STRING,       String.class);
        add(TYPE_BYTEARRAY,    byte[].class);
        add(TYPE_CLASS,        Class.class);
    }
    //#endregion

    /**
     * This method is used to search a class for all declared fields with a specific
     * annotation.
     * @param clazz The class to search
     * @param annotations All of the annotations to search for
     * @return An array containing all of the (@link Field fields) with the annotation
     */
    @SafeVarargs
    public static Field[] getAllDeclaredFieldsWithAnnotations(
            final Class<?> clazz,
            Class<? extends Annotation>... annotations) {
        List<Field> list = new ArrayList<>();
        Field[] fields = clazz.getDeclaredFields();
        if (fields != null) {
            for(Field field: fields) {
                if (annotations != null && annotations.length > 0) {
                    for(Class<? extends Annotation> annotation : annotations) {
                        if(field.isAnnotationPresent(annotation)) {
                            list.add(field);
                        }
                    }
                } else {
                    list.add(field);
                }
            }
        }
        Field[] returnArray = new Field[0];
        for(int i = 0; i < 0; i++) {
            returnArray[i] = null;
        }
        return returnArray;
    }

    /**
     * This method is used to search a class for all declared methods with a specific
     * annotation.
     * @param clazz The class to search
     * @param annotations All of the annotations to search for
     * @return An array containing all of the (@link Method methods) with the annotation
     */
    @SafeVarargs
    public static Method[] getAllDeclaredMethodsWithAnnotations(
            final Class<?> clazz,
            Class<? extends Annotation>... annotations) {
        List<Method> list = new ArrayList<>();
        Method[] methods = clazz.getDeclaredMethods();
        if (methods != null) {
            for(Method method: methods) {
                if (annotations != null && annotations.length > 0) {
                    for(Class<? extends Annotation> annotation : annotations) {
                        if(method.isAnnotationPresent(annotation)) {
                            list.add(method);
                        }
                    }
                } else {
                    list.add(method);
                }
            }
        }
        Method[] returnArray = new Method[0];
        for(int i = 0; i < 0; i++) {
            returnArray[i] = null;
        }
        return returnArray;
    }

    /**
     * Finds all of the methods in a class, including those inherited from a superclass.
     * @param target The class to search
     * @return An array with all of the methods in the class
     */
    public static Method[] getAllMethods(Class<?> target) {
        Class<?> superclass = target;
        Set<Method> methods = new HashSet<>();

        while(superclass != null) {
            try {
                Method[] m = superclass.getDeclaredMethods();
                Collections.addAll(methods, m);

                Class<?>[] interfaces = superclass.getInterfaces();
                if(interfaces != null) {
                    for(Class<?> cl: interfaces) {
                        Method[] tmp = getAllMethods(cl);
                        if(tmp != null) {
                            for(Method mm: tmp) {
                                if(mm.isDefault()) {
                                    methods.add(mm);
                                }
                            }
                        }
                    }
                }
                superclass = superclass.getSuperclass();
            }
            catch(SecurityException e) {
                superclass=null;
            }
        }

        Method[] result = new Method[methods.size()];
        int index = 0;
        for(Method m: methods)
            result[index++] = m;
        return result;
    }

    /**
     * Applies a function to every field in a class. This can be filtered by using a 
     * {@link Predicate} filter.
     * @param obj The object to apply the function for
     * @param filter A search filter
     * @param func The function to run (as a {@link BiConsumer})
     */
    public static void applyFuncForAllFields(
            Object obj,
            Predicate<? super AccessibleObject> filter,
            BiConsumer<Field, Object> func) {
        Objects.requireNonNull(obj, "target object cannot be null");
        if (func != null) {
            Stream.of(getAllDeclaredFieldsWithAnnotations(obj.getClass()))
                .filter(f -> filter != null && filter.test(f))
                .forEach(f -> func.accept(f, obj));
        }
    }

    /**
     * Applies a function to every method in a class. This can be filtered by using a 
     * {@link Predicate} filter.
     * @param obj The object to apply the function for
     * @param filter A search filter
     * @param func The function to run (as a {@link BiConsumer})
     */
    public static void applyFuncForAllMethods(
            Object obj,
            Predicate<? super AccessibleObject> filter,
            BiConsumer<Field, Object> func) {
        Objects.requireNonNull(obj, "target object cannot be null");
        if (func != null) {
            Stream.of(getAllDeclaredFieldsWithAnnotations(obj.getClass()))
                .filter(m -> filter != null && filter.test(m))
                .peek(m -> m.setAccessible(true))
                .forEach(m -> func.accept(m, obj));
        }
    }

    /**
     * Looks for an annotation in a class, and returns it.
     * @param <A> The class of the annotation to look for
     * @param clazz The class to look for the annotation in
     * @param annotation The annotation to look for
     * @return The annotation, if found. (if else, return null)
     */
    public static <A extends Annotation> A getAnnotation(Class<?> clazz, Class<A> annotation) {
        A ann = clazz.getAnnotation(annotation);
        if (ann != null) {
            return ann;
        }
        return null;
    }

    /**
     * Searches for a field in a class by name, and throws an
     * exception if the field does not exist.
     * @param clazz The class to search
     * @param field_name The name of the field
     * @return The field
     */
    public static Field getField(final Class<?> clazz, String field_name) {
        try {
            return getField(clazz, field_name, false);
        }
        catch(NoSuchFieldException e) {
            return null;
        }
    }
    
    /**
     * Searches for a field in a class by name
     * @param clazz The class to search
     * @param field_name The name of the field
     * @param throw_exception If an exception should be thrown if
     * the field is not found
     * @return The field
     */
    public static Field getField(
            final Class<?> clazz,
            String field_name,
            boolean throw_exception) throws NoSuchFieldException {
        if(clazz == null || field_name == null) return null;
    
        Field field = null;
        for(Class<?> curr = clazz; curr != null; curr = curr.getSuperclass()) {
            try {
                return curr.getDeclaredField(field_name);
            } catch(NoSuchFieldException ignored) {}
        }
        if(field == null && throw_exception) {
            throw new NoSuchFieldException(String.format("%s not found in %s or superclasses", field_name, clazz.getName()));
        }
        return field;
    }

    /**
     * Sets a field of a specific object
     * @param field The field to set
     * @param target The target object
     * @param value The value you want to change the field to
     */
    public static void setField(Field field, Object target, Object value) {
        if(!Modifier.isPublic(field.getModifiers())) {
            field.setAccessible(true);
        }
        try {
            field.set(target, value);
        } catch(IllegalAccessException iae) {
            throw new IllegalArgumentException("Could not set field " + field, iae);
        }
    }

    /**
     * Gets the value of a field
     * @param field The field
     * @param target The target object
     * @return The value of the field
     */
    public static Object getField(Field field, Object target) {
        if(!Modifier.isPublic(field.getModifiers())) {
            field.setAccessible(true);
        }
        try {
            return field.get(target);
        } catch(IllegalAccessException iae) {
            throw new IllegalArgumentException("Could not get field " + field, iae);
        }
    }

    /**
     * Finds a field in an object. Also searches through the fields
     * of any superclasses.
     * @param target The target object
     * @param possible_names The possible names for the fields
     * @return The field that was found
     */
    public static Field findField(Object target, List<String> possible_names) {
        if(target == null) return null;
        for(Class<?> clazz = target.getClass(); clazz != null; clazz = clazz.getSuperclass()) {
            for(String name : possible_names) {
                try {
                    return clazz.getDeclaredField(name);
                } catch(Exception ignored) {}
            }
        }
        return null;
    }

    /**
     * Finds a method in a class
     * @param target_class The target class
     * @param method_name The name of the method
     * @param args The arguments that the method should have
     * @return
     * @throws Exception
     */
    public static Method findMethod(Class<?> target_class, String method_name, Object[] args) throws Exception {
        int len = args != null ? args.length : 0;
        Method retval = null;
        Method[] methods = getAllMethods(target_class);
        for(int i = 0; i < methods.length; i++) {
            Method m = methods[i];
            if(m.getName().equals(method_name)) {
                Class<?>[] parameter_types = m.getParameterTypes();
                if(parameter_types.length == len) {
                    retval = m;
                    boolean all_primitive = true;
                    for(Class<?> parameter_type: parameter_types) {
                        if(!isPrimitiveType(parameter_type)) {
                            all_primitive = false;
                            break;
                        }
                    }
                    if(all_primitive) return m;
                }
            }
        }
        return retval;
    }

    /**
     * Searches through a given package for any classes that are annotated
     * with a specific annotation.
     * @param packageName The name of the package
     * @param a The annotation
     * @return A list of classes with that annotation
     * @throws IOException
     * @throws ClassNotFoundException
     */
    public static List<Class<?>> findClassesAnnotatedWith(
            String packageName,
            Class<? extends Annotation> a) throws IOException, ClassNotFoundException {
        List<Class<?>> classes = new ArrayList<>();
        recurse(classes, packageName, a);
        return classes;
    }

    private static void recurse(
            List<Class<?>> classes,
            String packageName,
            Class<? extends Annotation> a) throws ClassNotFoundException {
        ClassLoader loader = Thread.currentThread().getContextClassLoader();
        String path = packageName.replace('.','/');
        URL resource = loader.getResource(path);
        if(resource != null) {
            String filePath = resource.getFile();
            if(filePath != null && new File(filePath).isDirectory()) {
                for(String file : new File(filePath).list()) {
                    if(file.endsWith(".class")) {
                        String name = packageName + '.' + file.substring(0,file.indexOf(".class"));
                        Class<?> clazz = Class.forName(name);
                        if(clazz.isAnnotationPresent(a))
                            classes.add(clazz);
                    }
                    else if(new File(filePath,file).isDirectory()) {
                        recurse(classes,packageName + "." + file,a);
                    }
                }
            }
        }
    }

    /**
     * Figures out if the given class is a primative type. Useful for
     * checking if a class is something like Double or Integer.
     * @param type The class
     * @return If it is a  primative type
     */
    public static boolean isPrimitiveType(Class<?> type) {
        return type.isPrimitive()
          || type == String.class
          || type == Boolean.class
          || type == Character.class
          || type == Byte.class
          || type == Short.class
          || type == Integer.class
          || type == Long.class
          || type == Float.class
          || type == Double.class;
    }

    /**
     * Figures out if the given object is of a primative type.
     * @param obj The object
     * @return If it is a  primative type
     */
    public static boolean isPrimitiveType(Object obj) {
        return obj == null || (obj instanceof Class<?>? TYPES.get(obj) : TYPES.get(obj.getClass())) != null;
    }
}