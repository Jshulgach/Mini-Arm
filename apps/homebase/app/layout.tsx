import type { Metadata } from 'next';
import './globals.css';
import './presentation.css';
export const metadata: Metadata = {
  title: 'Homebase — Your home workspace',
  description:
    'A local workspace for your smart home, robots and device connections.',
  icons: { icon: '/favicon.svg' },
};
export default function RootLayout({
  children,
}: Readonly<{ children: React.ReactNode }>) {
  return (
    <html lang="en" className="dark">
      <body>{children}</body>
    </html>
  );
}
