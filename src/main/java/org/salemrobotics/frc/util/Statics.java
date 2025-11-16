/*
 * Copyright (c) 2025 The Blue Devils.
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 */
package org.salemrobotics.frc.util;

import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Consumer;
import java.util.function.Supplier;

/** Helper class for static initialization. */
public class Statics {
  /**
   * Equivalent to {@link Supplier}, however it may throw a checked exception.
   *
   * @param <T> The type of the value being returned.
   * @param <E> The type of the exception that can be thrown. Setting this to {@link Exception} then
   *     any type of exception may be thrown.
   */
  @FunctionalInterface
  public interface ThrowableSupplier<T, E extends Exception> {
    public T get() throws E;
  }

  /**
   * Attempts to initialize a value with one constructor, using a fallback if it throws an error.
   *
   * @param <T> The type of the value being initialized.
   * @param init The constructor that may throw an exception.
   * @param defaultSupplier The default value to use if {@code init} throws something. This will run
   *     unchecked, so throwing an exception in this function will cause your robot program to crash
   *     if it is not properly handled.
   * @return The initialized value.
   */
  public static final <T> T initOrDefault(
      ThrowableSupplier<T, Exception> init, Supplier<T> defaultSupplier) {
    return initOrDefault(init, defaultSupplier, Statics::defaultExceptionHandler);
  }

  /**
   * Attempts to initialize a value with one constructor, using a given exception handler and
   * default supplier if it throws an exception.
   *
   * @param <T> The type of the value being initialized.
   * @param <E> The type of exception that may be thrown.
   * @param init The constructor that may throw an exception.
   * @param defaultSupplier The default value to use if {@code init} throws something. This will run
   *     unchecked, so throwing an exception in this function will cause your robot program to crash
   *     if it is not properly handled.
   * @param exceptionHandler The exception handler to use if an exception is encountered.
   * @return The initialized value.
   */
  @SuppressWarnings("unchecked")
  public static final <T, E extends Exception> T initOrDefault(
      ThrowableSupplier<T, E> init, Supplier<T> defaultSupplier, Consumer<E> exceptionHandler) {
    try {
      return init.get();
    } catch (Throwable e) {
      exceptionHandler.accept((E) e);

      return defaultSupplier.get();
    }
  }

  /**
   * The default exception handler used by {@link Statics#initOrDefault(ThrowableSupplier,
   * Supplier)}. This reports the error to DriverStation with the error's filled-in stack trace.
   *
   * @param <E> The type of exception being handled (this does not affect how the exception is
   *     handled).
   * @param error The error to handle.
   */
  public static final <E extends Exception> void defaultExceptionHandler(E error) {
    Throwable throwable = error.fillInStackTrace();

    DriverStation.reportError("Error ocurred during static initialization: ", false);
    DriverStation.reportError(throwable.toString(), throwable.getStackTrace());
  }
}
