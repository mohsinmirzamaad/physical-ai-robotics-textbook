/**
 * Better-Auth server configuration
 * Handles authentication with Neon Postgres adapter and custom user fields
 */

import { betterAuth } from "better-auth";
import { Pool } from "pg";

// Create PostgreSQL connection pool
const pool = new Pool({
  connectionString: process.env.DATABASE_URL,
  ssl: {
    rejectUnauthorized: false
  }
});

export const auth = betterAuth({
  database: {
    provider: "postgres",
    pool: pool,
  },

  emailAndPassword: {
    enabled: true,
    requireEmailVerification: false, // Set to true in production with email service
  },

  session: {
    expiresIn: 60 * 60 * 24 * 7, // 7 days
    updateAge: 60 * 60 * 24, // 1 day
    cookieCache: {
      enabled: true,
      maxAge: 5 * 60, // 5 minutes
    },
  },

  user: {
    additionalFields: {
      softwareExperience: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        validate: (value: string) => {
          return ["beginner", "intermediate", "advanced"].includes(value);
        },
      },
      hardwareExperience: {
        type: "string",
        required: false,
        defaultValue: "beginner",
        validate: (value: string) => {
          return ["beginner", "intermediate", "advanced"].includes(value);
        },
      },
    },
  },

  advanced: {
    generateId: () => {
      // Use UUID v4 for consistency with database
      return crypto.randomUUID();
    },
  },

  trustedOrigins: [
    process.env.AUTH_URL || "http://localhost:3000",
  ],
});

export type Session = typeof auth.$Infer.Session;
export type User = typeof auth.$Infer.User;
