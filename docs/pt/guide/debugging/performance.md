---
description: Esta página explica técnicas de depuração de desempenho para aplicações RxJS. Fornece técnicas práticas como rastrear o número de subscriptions, detectar reavaliações desnecessárias, monitorar uso de memória, configurar o ambiente de desenvolvimento, depuração type-safe e definir error boundaries.
---

# Depuração de Desempenho e Melhores Práticas

Esta sessão cobrirá técnicas para otimizar o desempenho de aplicações RxJS e criar um ambiente de depuração eficiente.

## Verificar Contagem de Subscription

Verifique para ver se múltiplas subscriptions foram criadas não intencionalmente.

```ts
import { Observable, defer } from 'rxjs';
import { finalize } from 'rxjs';

let globalSubscriptionId = 0;
let activeSubscriptions = 0;

/**
 * Operador personalizado para rastrear contagem de subscription
 */
function tracked<T>(label: string) {
  return (source: Observable<T>) =>
    defer(() => {
      const id = ++globalSubscriptionId;
      activeSubscriptions++;
      console.log(`➕ Subscription iniciada [${label}] #${id} (Ativo: ${activeSubscriptions})`);

      return source.pipe(
        finalize(() => {
          activeSubscriptions--;
          console.log(`➖ Subscription finalizada [${label}] #${id} (Ativo: ${activeSubscriptions})`);
        })
      );
    });
}

// Exemplo de uso
import { interval } from 'rxjs';
import { take } from 'rxjs';

const stream$ = interval(1000).pipe(
  take(3),
  tracked('Test Stream')
);

const sub1 = stream$.subscribe();
const sub2 = stream$.subscribe();

setTimeout(() => {
  sub1.unsubscribe();
  sub2.unsubscribe();
}, 5000);

// Saída:
// ➕ Subscription iniciada [Test Stream] #1 (Ativo: 1)
// ➕ Subscription iniciada [Test Stream] #2 (Ativo: 2)
// ➖ Subscription finalizada [Test Stream] #1 (Ativo: 1)
// ➖ Subscription finalizada [Test Stream] #2 (Ativo: 0)
```

Nesta implementação:
- ✅ `defer` para gerar um novo ID cada vez que você faz subscribe
- ✅ `finalize` para garantir que o processo de unsubscription seja executado de forma confiável
- ✅ Rastreie o número de subscriptions ativas em tempo real
- ✅ Type safe e funciona com RxJS v8

## Detectar reavaliação desnecessária

Verifica para ver se o mesmo valor foi calculado mais de uma vez.

```ts
import { of } from 'rxjs';
import { map, tap, shareReplay } from 'rxjs';

let computeCount = 0;

function expensiveComputation(value: number): number {
  computeCount++;
  console.log(`💰 Computação executada (${computeCount} vezes):`, value);
  // Simular computação pesada
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result += Math.sin(i);
  }
  return result;
}

// ❌ Sem shareReplay → Computado para cada subscription
console.log('=== Sem shareReplay ===');
computeCount = 0;
const withoutShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x))
);

withoutShare$.subscribe(v => console.log('Subscription 1:', v));
withoutShare$.subscribe(v => console.log('Subscription 2:', v));
// Saída: Computação executa 6 vezes (3 valores × 2 subscriptions)

// ✅ Com shareReplay → Resultados de computação são compartilhados
console.log('\n=== Com shareReplay ===');
computeCount = 0;
const withShare$ = of(1, 2, 3).pipe(
  map(x => expensiveComputation(x)),
  shareReplay(3)
);

withShare$.subscribe(v => console.log('Subscription 1:', v));
withShare$.subscribe(v => console.log('Subscription 2:', v));
// Saída: Computação executa apenas 3 vezes
```

## Monitorar uso de memória

Este método de monitoramento é usado para detectar vazamentos de memória.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MemoryMonitor {
  private intervals: ReturnType<typeof setInterval>[] = [];

  start(intervalMs: number = 5000) {
    const id = setInterval(() => {
      if (typeof performance !== 'undefined' && (performance as any).memory) {
        const memory = (performance as any).memory;
        console.log('📊 Uso de memória:', {
          Usado: `${(memory.usedJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Total: `${(memory.totalJSHeapSize / 1024 / 1024).toFixed(2)} MB`,
          Limite: `${(memory.jsHeapSizeLimit / 1024 / 1024).toFixed(2)} MB`
        });
      }
    }, intervalMs);

    this.intervals.push(id);
  }

  stop() {
    this.intervals.forEach(id => clearInterval(id));
    this.intervals = [];
  }
}

// Exemplo de uso
const monitor = new MemoryMonitor();
monitor.start(5000); // Exibir uso de memória a cada 5 segundos

// Testar vazamento de memória
const leakyStreams: any[] = [];

for (let i = 0; i < 100; i++) {
  // ❌ Stream sem unsubscription
  const sub = interval(100).subscribe();
  leakyStreams.push(sub);
}

// Fazer unsubscribe após 10 segundos
setTimeout(() => {
  console.log('Unsubscription iniciada');
  leakyStreams.forEach(sub => sub.unsubscribe());
  console.log('Unsubscription completada');

  // Parar monitoramento após outros 10 segundos
  setTimeout(() => {
    monitor.stop();
  }, 10000);
}, 10000);
```

## Melhores Práticas

### Estabelecendo um Ambiente de Depuração

Como habilitar logging de debug apenas no ambiente de desenvolvimento.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Determinar modo debug (ajuste de acordo com a ferramenta de build)
const IS_DEVELOPMENT =
  // Ao usar Vite: import.meta.env.DEV
  // Ao usar webpack: process.env.NODE_ENV === 'development'
  // Configuração manual: defina variável global
  typeof window !== 'undefined' && (window as any).__DEV__ === true;

function devLog<T>(label: string) {
  if (!IS_DEVELOPMENT) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => console.log(`[${label}]`, value),
    error: error => console.error(`[${label}] Erro:`, error),
    complete: () => console.log(`[${label}] Complete`)
  });
}

// Exemplo de uso
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    devLog('Entrada'),
    map(x => x * 2),
    devLog('Saída')
  )
  .subscribe();
// Sem logs no ambiente de produção
```

### Depuração type safe

Este é um método de depuração que aproveita o sistema de tipos TypeScript.

```ts
import { tap } from 'rxjs';

type LogLevel = 'debug' | 'info' | 'warn' | 'error';

interface TypedDebugOptions<T> {
  label: string;
  level?: LogLevel;
  transform?: (value: T) => any;
  filter?: (value: T) => boolean;
}

function typedDebug<T>(options: TypedDebugOptions<T>) {
  const { label, level = 'debug', transform, filter } = options;

  const logFn = console[level] || console.log;

  return tap<T>({
    next: value => {
      if (filter && !filter(value)) return;

      const displayValue = transform ? transform(value) : value;
      logFn(`[${label}]`, displayValue);
    }
  });
}

// Exemplo de uso
interface User {
  id: number;
  name: string;
  email: string;
}

import { of } from 'rxjs';

of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob', email: 'bob@example.com' },
  { id: 3, name: 'Charlie', email: 'charlie@example.com' }
)
  .pipe(
    typedDebug<User>({
      label: 'User Stream',
      level: 'info',
      transform: user => `${user.name} (${user.email})`,
      filter: user => user.id > 1
    })
  )
  .subscribe();

// Saída:
// [User Stream] Bob (bob@example.com)
// [User Stream] Charlie (charlie@example.com)
```

### Definindo Error Boundaries

Isole erros adequadamente para facilitar a depuração.

```ts
import { Observable, of } from 'rxjs';
import { catchError } from 'rxjs';

function errorBoundary<T>(label: string) {
  return (source: Observable<T>) =>
    source.pipe(
      catchError(error => {
        console.error(`🔴 [${label}] Erro capturado:`, {
          message: error.message,
          stack: error.stack,
          timestamp: new Date().toISOString()
        });

        // Relançar erro ou retornar valor de fallback
        throw error;
      })
    );
}

// Exemplo de uso
import { throwError } from 'rxjs';
import { mergeMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    errorBoundary('Processo principal'),
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Erro no valor 2'));
      }
      return of(value);
    }),
    errorBoundary('Processo assíncrono')
  )
  .subscribe({
    next: value => console.log('Sucesso:', value),
    error: error => console.log('Erro final:', error.message)
  });
```

## Resumo

Depuração de Desempenho e Melhores Práticas:

### Monitoramento de Desempenho
- ✅ **Rastrear subscriptions** - gerenciar subscriptions usando defer e finalize
- ✅ **Detectar reavaliações** - evitar cálculos desnecessários com shareReplay
- ✅ **Monitoramento de Memória** - rastrear uso de memória com performance API

### Otimizar seu ambiente de desenvolvimento
- ✅ **Configurações específicas do ambiente** - habilitar logging de debug apenas no ambiente de desenvolvimento
- ✅ **Depuração type safe** - aproveitar o sistema de tipos TypeScript
- ✅ **Error Boundaries** - isolar e depurar erros adequadamente

Juntas, essas técnicas otimizam o desempenho de aplicações RxJS e criam um ambiente de depuração eficiente.

## Páginas Relacionadas

- [Estratégias Básicas de Depuração](/pt/guide/debugging/) - Como usar operador tap e ferramentas do desenvolvedor
- [Cenários Comuns de Depuração](/pt/guide/debugging/common-scenarios) - Solução de problemas específicos
- [Ferramentas de Debug Personalizadas](/pt/guide/debugging/custom-tools) - Streams nomeados, operadores de debug
- [Operador - shareReplay](/pt/guide/operators/multicasting/shareReplay) - Evitar reavaliações desnecessárias
