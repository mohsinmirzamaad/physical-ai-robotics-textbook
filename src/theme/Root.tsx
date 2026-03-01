/**
 * Root component wrapper for Docusaurus
 * Provides SessionProvider to all components
 */

import React from "react";
import { authClient } from "@site/src/lib/auth-client";

// Root component that wraps the entire Docusaurus app
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <authClient.SessionProvider>
      {children}
    </authClient.SessionProvider>
  );
}
