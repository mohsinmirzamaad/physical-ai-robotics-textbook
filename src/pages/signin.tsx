import React from 'react';
import Layout from '@theme/Layout';
import SigninForm from '@site/src/components/Auth/SigninForm';
import { useHistory } from '@docusaurus/router';

export default function SigninPage() {
  const history = useHistory();

  const handleSuccess = () => {
    // Redirect to home page after successful signin
    history.push('/');
  };

  return (
    <Layout title="Sign In" description="Sign in to your account">
      <SigninForm onSuccess={handleSuccess} />
    </Layout>
  );
}
