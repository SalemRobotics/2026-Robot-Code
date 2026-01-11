package com.frc6324.lib;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/**
 * Marks a 'static' class - that is, a class that cannot be instantiated. This enforces certain
 * patterns to prevent such an action from happening.
 *
 * <p>For example, the following code:
 *
 * <pre>{@code
 * @UninstantiableClass
 *
 * public class MyStaticClass {
 *     // ...
 * }
 * }</pre>
 *
 * will fail to compile, as the {@code MyStaticClass} is not marked {@code final} and can therefore
 * be subclassed and instantiated.
 *
 * <p>This annotation <em>cannot</em> ensure that the constructor is reflection-resistant; to do so,
 * the constructor should look like:
 *
 * <pre>{@code
 * private MyStaticClass() {
 *     throw new IllegalStateException();
 * }
 * }</pre>
 */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE)
public @interface UninstantiableClass {}
