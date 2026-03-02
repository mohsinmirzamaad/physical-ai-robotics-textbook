import React from 'react';
import Layout from '@theme/Layout';
import SignupForm from '@site/src/components/Auth/SignupForm';
import { useHistory } from '@docusaurus/router';

export default function SignupPage() {
  const history = useHistory();

  const handleSuccess = () => {
    // Redirect to home page after successful signup
    history.push('/');
  };

  return (
    <Layout title="Sign Up" description="Create your account">
      <SignupForm onSuccess={handleSuccess} />
    </Layout>
  );
}
