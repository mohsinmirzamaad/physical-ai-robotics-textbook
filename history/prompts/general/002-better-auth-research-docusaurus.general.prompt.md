---
id: 002
title: Better-Auth Research for Docusaurus Authentication
stage: general
date: 2026-03-01
surface: agent
model: claude-opus-4-6
feature: none
branch: main
user: Mohsin MM
command: research
labels: ["authentication", "better-auth", "docusaurus", "neon-postgres", "research"]
links:
  spec: null
  ticket: null
  adr: null
  pr: null
files:
tests:
---

## Prompt

Research Better-Auth library for authentication in a Docusaurus-based application:

1. Latest Better-Auth version and compatibility with React/Docusaurus
2. Setup process - configuration, providers, session management
3. Integration with Neon Serverless Postgres - schema requirements, connection setup
4. Custom fields during signup (background questionnaire) - how to extend the signup flow
5. Session persistence across page navigation in a static site
6. Password reset flow implementation
7. Security best practices (password hashing, session tokens, CSRF protection)

Context: Need email/password authentication with custom signup questions (software/hardware experience levels). Users must stay logged in across chapter navigation.

Provide implementation guidance with code patterns.

## Response snapshot

Comprehensive research completed on Better-Auth library integration with Docusaurus:

**Key Findings**:
- Better-Auth v1.3.10 (latest as of March 2026) is framework-agnostic and fully compatible with React/Docusaurus
- Provides dedicated React hooks via `better-auth/react` including `useSession` for client-side session management
- Native support for Neon Serverless Postgres through standard PostgreSQL Pool adapter
- Custom user fields supported via `additionalFields` configuration with full TypeScript inference
- Session persistence handled automatically via HTTP-only cookies with built-in CSRF protection
- Password reset flow includes token-based email verification with configurable expiration
- Security features include bcrypt password hashing, secure session tokens, rate limiting, and enumeration protection

**Implementation Patterns Provided**:
1. Server configuration with Neon Postgres connection
2. Extended user schema with custom questionnaire fields (softwareExperience, hardwareExperience, background)
3. React signup component with custom field collection
4. Session persistence setup for Docusaurus static site using SessionProvider
5. Protected route wrapper for chapter navigation
6. Complete password reset flow (request + reset components)
7. Security hardening configuration with environment variables

**Database Schema**: Better-Auth CLI automatically manages migrations with core tables (user, session, account, verification) plus custom fields.

**Session Strategy**: HTTP-only cookies with automatic persistence across navigation, no additional client-side storage needed.

## Outcome

- ✅ Impact: Complete implementation guide for Better-Auth + Docusaurus + Neon Postgres authentication system with custom signup fields
- 🧪 Tests: No tests run (research phase)
- 📁 Files: PHR created at D:\AgenticAI\hackathon-1\history\prompts\general\002-better-auth-research-docusaurus.general.prompt.md
- 🔁 Next prompts: Implementation phase - create auth configuration, database schema, signup/login components
- 🧠 Reflection: Better-Auth provides excellent TypeScript support and framework-agnostic design making it ideal for Docusaurus static sites. Custom field extension is straightforward and type-safe.

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (research task)
- Prompt variant (if applicable): N/A
- Next experiment (smallest change to try): Begin implementation with server-side auth configuration and Neon database setup
