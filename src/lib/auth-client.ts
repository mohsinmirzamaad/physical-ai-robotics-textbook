/**
 * Better-Auth client hooks for React components
 * Provides authentication state and actions
 */

import { createAuthClient } from "better-auth/react";

export const authClient = createAuthClient({
  baseURL: process.env.AUTH_URL || "http://localhost:3000",
});

// Export hooks for use in components
export const {
  useSession,
  signIn,
  signUp,
  signOut,
  useActiveOrganization,
} = authClient;

/**
 * Sign up a new user with email and password
 * @param email User email
 * @param password User password
 * @param name User name (optional)
 * @param softwareExperience Software experience level
 * @param hardwareExperience Hardware experience level
 */
export async function signUpUser(
  email: string,
  password: string,
  name?: string,
  softwareExperience: "beginner" | "intermediate" | "advanced" = "beginner",
  hardwareExperience: "beginner" | "intermediate" | "advanced" = "beginner"
) {
  return await signUp.email({
    email,
    password,
    name,
    callbackURL: "/",
    data: {
      softwareExperience,
      hardwareExperience,
    },
  });
}

/**
 * Sign in an existing user with email and password
 * @param email User email
 * @param password User password
 */
export async function signInUser(email: string, password: string) {
  return await signIn.email({
    email,
    password,
    callbackURL: "/",
  });
}

/**
 * Sign out the current user
 */
export async function signOutUser() {
  return await signOut({
    fetchOptions: {
      onSuccess: () => {
        window.location.href = "/";
      },
    },
  });
}

/**
 * Get the current session
 */
export function useCurrentSession() {
  const session = useSession();
  return {
    user: session.data?.user,
    isLoading: session.isPending,
    isAuthenticated: !!session.data?.user,
    session: session.data?.session,
  };
}
