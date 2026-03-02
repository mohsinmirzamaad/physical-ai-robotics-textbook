/**
 * Root component wrapper for Docusaurus
 * Provides SessionProvider and ChatbotWidget to all components
 */

import React from "react";
import ChatbotWidget from "@site/src/components/ChatbotWidget";

// Root component that wraps the entire Docusaurus app
// TODO: Add SessionProvider when auth is fully configured
export default function Root({ children }: { children: React.ReactNode }) {
  return (
    <>
      {children}
      <ChatbotWidget />
    </>
  );
}
