
import { useEffect, useRef } from 'react';
import { Card } from "@/components/ui/card";
import { ScrollArea } from "@/components/ui/scroll-area";

interface Message {
  timestamp: number;
  content: string;
  type: 'info' | 'error' | 'warning';
}

interface SerialMonitorProps {
  messages: Message[];
}

export function SerialMonitor({ messages }: SerialMonitorProps) {
  const scrollAreaRef = useRef<HTMLDivElement>(null);

  useEffect(() => {
    if (scrollAreaRef.current) {
      const scrollArea = scrollAreaRef.current;
      scrollArea.scrollTop = scrollArea.scrollHeight;
    }
  }, [messages]);

  const getMessageColor = (type: Message['type']) => {
    switch (type) {
      case 'error':
        return 'text-red-400';
      case 'warning':
        return 'text-yellow-400';
      default:
        return 'text-green-400';
    }
  };

  return (
    <Card className="data-card h-[450px]">
      <div className="flex items-center justify-between mb-2">
        <h3 className="text-lg font-semibold">Serial Monitor</h3>
        <div className="flex space-x-2">
          <div className="h-2 w-2 rounded-full bg-green-500 animate-pulse" />
          <span className="text-sm text-muted-foreground">Connected</span>
        </div>
      </div>
      <ScrollArea className="h-[380px] rounded-md border" ref={scrollAreaRef}>
        <div className="p-4 font-mono text-sm">
          {messages.map((msg, index) => (
            <div key={index} className={`${getMessageColor(msg.type)} mb-1`}>
              <span className="text-muted-foreground">
                {new Date(msg.timestamp).toLocaleTimeString()} &gt;
              </span>{' '}
              {msg.content}
            </div>
          ))}
        </div>
      </ScrollArea>
    </Card>
  );
}
