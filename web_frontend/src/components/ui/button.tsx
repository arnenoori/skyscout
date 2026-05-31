import * as React from 'react';
import { Slot } from '@radix-ui/react-slot';

import {
  buttonVariants,
  type ButtonVariantProps,
} from '@/components/ui/button-variants';
import { cn } from '@/lib/utils';

interface ButtonProps
  extends React.ComponentPropsWithRef<'button'>, ButtonVariantProps {
  asChild?: boolean;
}

function Button({
  className,
  variant,
  size,
  asChild = false,
  type = 'button',
  ref,
  ...props
}: ButtonProps) {
  const Comp = asChild ? Slot : 'button';

  return (
    <Comp
      className={cn(buttonVariants({ variant, size, className }))}
      ref={ref}
      type={asChild ? undefined : type}
      {...props}
    />
  );
}

export { Button };
