package edu.illinois.mitra.starl.models;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
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
    private static final Map<String, Constructor<? extends Model>> map = new HashMap<>();

    /**
     * Registers a subclass of Model to be used in the application.
     *
     * @throws RuntimeException if the type has already been registered or has no default constructor
     * @param c a Class object representing the type to be registered
     */
    public static void register(Class<? extends Model> c) {
        check();
        synchronized(lock) {
            check();

            // Model.getTypeName() implementation must return the same string
            String typeName = c.getSimpleName();
            if (map.containsKey(typeName)) {
                throw new RuntimeException(typeName + " is already registered.");
            }

            Constructor<? extends Model> modelConstructor;
            try {
                modelConstructor = c.getConstructor();
            } catch (NoSuchMethodException e) {
                throw new RuntimeException(typeName + " has no default constructor. " + e);
            }

            map.put(typeName, modelConstructor);
            System.out.println("Registered " + typeName);
        }
    }

    /**
     * Instantiates a subclass of Model given by the typeName parameter.
     * @param typeName the type of Model to instantiate
     * @return a new instance of class typeName
     * @throws RuntimeException if typeName is not registered or if construction fails
     */
    public static Model create(String typeName) throws RuntimeException {
        access(); // make read-only

        Constructor<? extends Model> modelConstructor = map.get(typeName);
        if (modelConstructor == null) {
            throw new RuntimeException(typeName + " not found in registry.");
        }
        Model model;
        try {
            model = modelConstructor.newInstance();
        } catch (InstantiationException | IllegalAccessException
                | IllegalArgumentException | InvocationTargetException e) {
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
     * @return a set of typenames
     */
    public static Set<String> types() {
        access(); // make read-only
        return map.keySet();
    }

    // make sure registry has not been accessed (use in double-checked locking)
    private static void check() {
        if (accessed) {
            throw new IllegalStateException("Cannot modify registry after accessing it.");
        }
    }

    // mark the registry as accessed
    private static void access() {
        if (!accessed) {
            synchronized (lock) {
                accessed = true;
            }
        }
    }
}
