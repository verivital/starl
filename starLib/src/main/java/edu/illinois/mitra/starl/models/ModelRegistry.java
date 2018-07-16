package edu.illinois.mitra.starl.models;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

/**
 * ModelRegistry is a static container class whose purpose is to eliminate
 * explicit typing of the concrete Model subclasses. Each concrete Model subclass
 * will call <code>ModelRegistry.register(Model_Something.class);</code>
 * in their static initializers to register themselves for use in the rest of the
 * application. The first time the registry is read, it disables all further
 * modification and becomes immutable, essentially like a global constant.
 */
public class ModelRegistry {

    private static final Object lock = new Object();
    private static volatile boolean accessed = false; // volatile for atomic reads and writes
    private static final Map<String, Class<? extends Model>> map = new HashMap<>();

    /**
     * Registers a subclass of Model to be used in the application.
     *
     * @throws RuntimeException if the type has already been registered or has no default constructor
     * @param c a Class object representing the type to be registered
     */
    public static void register(Class<? extends Model> c) {
        String typeName = c.getSimpleName();
        check(); // first check
        synchronized(lock) {
            check(); // second check in double-checked locking
            if (map.containsKey(typeName)) {
                throw new RuntimeException(typeName + " is already registered.");
            }

            map.put(typeName, c);
        }
        System.out.println("Registered " + typeName);
    }

    /**
     * Instantiates a subclass of Model given by the typeName parameter. Although this is
     * a more expensive operation than a raw <code>new</code>, it enables Models to be created
     * with any arguments without knowing the explicit type.
     *
     * @param typeName the type of Model to instantiate
     * @param args the arguments to the Model constructor
     * @return a new instance of class typeName
     * @throws RuntimeException if typeName is not registered or if construction fails
     */
    public static Model create(String typeName, Object... args) throws RuntimeException {
        access(); // make read-only

        Class<? extends Model> modelClass;
        Class<?>[] formalTypes;
        Constructor<? extends Model> modelConstructor;
        Model model;

        // load the registered class
        modelClass = map.get(typeName);
        if (modelClass == null) {
            throw new RuntimeException(typeName + " not found in registry.");
        }

        // populate an array with the Classes of the arguments
        formalTypes = new Class<?>[args.length];
        for (int i = 0; i < args.length; i++) {
            formalTypes[i] = args[i].getClass();
        }

        try {
            // attempt to get the constructor that can accept the arguments
            modelConstructor = modelClass.getDeclaredConstructor(formalTypes);
        } catch (NoSuchMethodException | SecurityException e) {
            throw new RuntimeException(typeName + " has no corresponding constructor to "
                    + Arrays.toString(formalTypes) + ". " + e);
        }

        try {
            // call the constructor with the arguments
            model = modelConstructor.newInstance(args);
        } catch (InstantiationException | IllegalAccessException
                | IllegalArgumentException | InvocationTargetException
                | ExceptionInInitializerError e) {
            throw new RuntimeException("Could not get new instance of " + typeName + ". " + e);
        }
        return model;
    }

    /**
     * Checks whether a type is in the registry.
     * @param typeName the type of Model to be looked up in the registry
     * @return whether create() will return successfully, barring a construction failure
     */
    public static boolean canCreate(String typeName) {
        access(); // make read-only
        return map.containsKey(typeName);
    }

    /**
     * Lists the types contained in the registry.
     * @return an unmodifiable set of typenames
     */
    public static Set<String> types() {
        access(); // make read-only
        return Collections.unmodifiableSet(map.keySet());
    }

    // make sure registry has not been accessed (use in double-checked locking)
    private static void check() {
        if (accessed) {
            throw new IllegalStateException("Cannot modify registry after accessing it.");
        }
    }

    // mark the registry as accessed, make read-only
    private static void access() {
        if (!accessed) {
            synchronized (lock) {
                accessed = true;
            }
        }
    }
}
