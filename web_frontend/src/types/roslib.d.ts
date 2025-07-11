declare module 'roslib' {
  export class Ros {
    constructor(options: { url: string });
    on(event: 'connection' | 'error' | 'close', callback: (error?: Error) => void): void;
    close(): void;
  }

  export class Topic {
    constructor(options: {
      ros: Ros;
      name: string;
      messageType: string;
    });
    publish(message: Message): void;
    subscribe(callback: (message: Record<string, unknown>) => void): void;
    unsubscribe(callback: (message: Record<string, unknown>) => void): void;
  }

  export class Message {
    constructor(values: Record<string, unknown>);
  }

  export class Service {
    constructor(options: {
      ros: Ros;
      name: string;
      serviceType: string;
    });
    callService(request: ServiceRequest, callback: (result: Record<string, unknown>) => void): void;
  }

  export class ServiceRequest {
    constructor(values: Record<string, unknown>);
  }
}
