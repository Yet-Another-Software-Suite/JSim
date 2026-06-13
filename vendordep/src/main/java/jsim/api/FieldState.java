// Copyright (c) JSim contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the LGPLv3 license file in the root directory of this project.

package jsim.api;

/**
 * Mutable wrapper for tracked simulation state.
 *
 * @param <T> tracked state type
 */
public class FieldState<T> {
    /** Wrapped state object. */
    private T state;

    /**
     * Creates a wrapper around the supplied state.
     *
     * @param state state object to store
     */
    public FieldState(T state) {
        this.state = state;
    }

    /**
     * Returns the wrapped state.
     *
     * @return wrapped state
     */
    public T get() {
        return state;
    }

    /**
     * Returns the wrapped state.
     *
     * @return wrapped state
     */
    public T getState() {
        return state;
    }

    /**
     * Replaces the wrapped state.
     *
     * @param state replacement state
     */
    public void set(T state) {
        this.state = state;
    }

    /**
     * Replaces the wrapped state.
     *
     * @param state replacement state
     */
    public void setState(T state) {
        this.state = state;
    }
}
