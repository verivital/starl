package edu.illinois.mitra.starl.models;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * ModelRegistry is a static container class whose purpose is to reduce
 * explicit typing of the concrete Model subclasses. Each concrete Model subclass
 * must be registered once in the static initializer for use in the rest of
 * the application. The first time the registry is read, it disables all further
 * modification and becomes immutable, essentially like a global constant,
 * thus eliminating the drawbacks of traditional singletons.
 */
public class ModelRegistry {

    private static final Object lock = new Object();
    private static volatile boolean accessed = false; // volatile for atomic reads and writes
    private static final Map<String, Class<? extends Model>> map = new HashMap<>();
    private static final Map<Class<?>, Class<?>> primitiveClasses = new HashMap<>();

    /*
     * To enable a type of Model to be used in the application, you must register it here.
     * Simply call register on one or more of the Classes of the concrete Model subclasses.
     */
    static {
        register(
                Model_3DR.class,
                Model_GhostAerial.class,
                Model_iRobot.class,
                Model_Mavic.class,
                Model_Phantom.class,
                Model_quadcopter.class
        );
    }

    /**
     * Registers one or more subclasses of Model to be used in the application. Safe to call repeatedly.
     *
     * Registering subclasses is only allowed before any data has been read from the registry,
     * through the methods {@link #create(String, Object...)}, {@link #canCreate(String)}, etc.
     *
     * @param classes one or more Class objects representing the type(s) to be registered
     * @throws RuntimeException if the type has already been registered or is abstract
     * @see Model
     */
    @SafeVarargs
    public static void register(Class<? extends Model>... classes) {
        check(); // first check
        synchronized(lock) {
            check(); // second check in double-checked locking
            for (Class<? extends Model> c : classes) {
                String typeName = c.getSimpleName();
                if (Modifier.isAbstract(c.getModifiers())) {
                    throw new IllegalArgumentException(typeName + " is abstract.");
                } else if (map.containsKey(typeName)) {
                    throw new IllegalArgumentException(typeName + " is already registered.");
                }
                map.put(typeName, c);
            }
        }
    }

    /**
     * Instantiates a subclass of Model given by the typeName parameter. Although this is
     * a more expensive operation than a raw <code>new</code>, it enables Models to be created
     * with any arguments without knowing the explicit type.
     * <p>
     * Waits for concurrent {link #register()} calls to complete and disables future
     * {link #register()} calls.
     * <p>
     * Note: because primitive types are boxed in variadic args, they are unboxed before calling
     * the constructor. This means that any constructor that takes a boxed type (Integer, etc.)
     * will not be found; prefer using primitive types.
     *
     * @param typeName the type of Model to instantiate
     * @param args the arguments to the Model constructor
     * @return a new instance of class typeName
     * @throws RuntimeException if typeName is not registered or if construction fails
     * @see Model
     */
    public static Model create(String typeName, Object... args) throws RuntimeException {
        access(); // make read-only

        // load the registered class
        Class<? extends Model> modelClass = loadFromRegistry(typeName);

        // populate an array with the Classes of the arguments
        Class<?>[] formalTypes = new Class<?>[args.length];
        for (int i = 0; i < args.length; i++) {
            // assume primitive types were intended instead of boxed object types
            formalTypes[i] = makePrimitive(args[i].getClass());
        }

        Constructor<? extends Model> modelConstructor;
        try {
            // attempt to get the constructor that can accept the arguments
            modelConstructor = modelClass.getDeclaredConstructor(formalTypes);
        } catch (NoSuchMethodException | SecurityException e) {
            throw new RuntimeException(typeName + " has no corresponding constructor to "
                    + Arrays.toString(formalTypes) + ". ", e);
        }

        Model model;
        try {
            // call the constructor with the arguments
            model = modelConstructor.newInstance(args);
        } catch (InstantiationException | IllegalAccessException | IllegalArgumentException
                | InvocationTargetException | ExceptionInInitializerError e) {
            throw new RuntimeException("Could not get new instance of " + typeName + ". ", e);
        }
        return model;
    }

    /**
     * Checks whether a type is in the registry.
     * <p>
     * Waits for concurrent {link #register()} calls to complete and disables future
     * {link #register()} calls.
     *
     * @param typeName the type of Model to be looked up in the registry
     * @return whether {@link #create(String, Object...)} will return successfully, barring a construction failure
     */
    public static boolean canCreate(String typeName) {
        access(); // make read-only
        return map.containsKey(typeName);
    }

    /**
     * Checks whether a type is or is a subclass of the given class object.
     * <p>
     * Waits for concurrent {link #register()} calls to complete and disables future
     * {link #register()} calls.
     *
     * @param typeName the type of Model to be looked up in the registry
     * @param c the class object to check to be a superclass
     * @return whether the class represented by typeName is an instance of the class represented by c
     * @throws IllegalArgumentException if typeName is not registered
     */
    public static boolean isInstance(String typeName, Class<? extends Model> c) {
        access(); // make read-only
        // load the registered class
        Class<? extends Model> modelClass = loadFromRegistry(typeName);
        return c.isAssignableFrom(modelClass);
    }

    /**
     * Lists the types contained in the registry.
     * <p>
     * Waits for concurrent {link #register()} calls to complete and disables future
     * {link #register()} calls.
     *
     * @return an unmodifiable set of typenames
     */
    public static Set<String> getTypes() {
        access(); // make read-only
        return Collections.unmodifiableSet(map.keySet());
    }

    // make sure registry has not been accessed (use in double-checked locking)
    private static void check() {
        if (accessed) {
            throw new IllegalStateException("Cannot modify registry after accessing it.");
        }
    }

    // mark the registry as accessed, make read-only. Only locks before accessed is set to true.
    private static void access() {
        if (!accessed) {
            synchronized (lock) {
                accessed = true;
            }
        }
    }

    // if given a Class object representing the object versions of the primitive types, return
    // the Class object of the corresponding primitive type
    static {
        primitiveClasses.put(Boolean.class, boolean.class);
        primitiveClasses.put(Byte.class, byte.class);
        primitiveClasses.put(Character.class, char.class);
        primitiveClasses.put(Double.class, double.class);
        primitiveClasses.put(Float.class, float.class);
        primitiveClasses.put(Integer.class, int.class);
        primitiveClasses.put(Long.class, long.class);
        primitiveClasses.put(Short.class, short.class);
    }

    private static Class<?> makePrimitive(Class<?> c) {
        Class<?> temp = primitiveClasses.get(c);
        return temp != null ? temp : c;
    }

    // load the Class object registered by typeName, or throw a IllegalArgumentException if not found
    private static Class<? extends Model> loadFromRegistry(String typeName) {
        // load the registered class
        Class<? extends Model> modelClass = map.get(typeName);
        if (modelClass == null) {
            throw new IllegalArgumentException(typeName + " not found in registry.");
        }
        return modelClass;
    }
}
