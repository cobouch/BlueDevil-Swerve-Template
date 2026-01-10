package com.frc6324.lib;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/** Marks a 'static' class - that is, a class that cannot be instantiated. */
@Target(ElementType.TYPE)
@Retention(RetentionPolicy.SOURCE)
public @interface UninstantiableClass {}
