import * as React from 'react';

import { cn } from '@/lib/utils';

function Card({
  className,
  ref,
  ...props
}: React.ComponentPropsWithRef<'div'>) {
  return (
    <div
      ref={ref}
      className={cn(
        'rounded-xl border bg-card/95 backdrop-blur supports-[backdrop-filter]:bg-card/60 text-card-foreground shadow-lg hover:shadow-xl transition-all duration-200',
        className
      )}
      {...props}
    />
  );
}

function CardHeader({
  className,
  ref,
  ...props
}: React.ComponentPropsWithRef<'div'>) {
  return (
    <div
      ref={ref}
      className={cn(
        'flex flex-col space-y-1.5 p-6 border-b border-border/50',
        className
      )}
      {...props}
    />
  );
}

function CardTitle({
  className,
  ref,
  ...props
}: React.ComponentPropsWithRef<'div'>) {
  return (
    <div
      ref={ref}
      className={cn(
        'font-semibold leading-none tracking-tight text-lg',
        className
      )}
      {...props}
    />
  );
}

function CardContent({
  className,
  ref,
  ...props
}: React.ComponentPropsWithRef<'div'>) {
  return <div ref={ref} className={cn('p-6 pt-0', className)} {...props} />;
}

export { Card, CardHeader, CardTitle, CardContent };
