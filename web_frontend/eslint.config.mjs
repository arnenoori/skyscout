import next from 'eslint-config-next';
import nextTypescript from 'eslint-config-next/typescript';

const eslintConfig = [
  {
    ignores: [
      '.next/**',
      'out/**',
      'build/**',
      'coverage/**',
      'next-env.d.ts',
      '*.tsbuildinfo',
    ],
  },
  ...next,
  ...nextTypescript,
];

export default eslintConfig;
