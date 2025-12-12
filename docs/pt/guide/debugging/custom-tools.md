---
description: Esta página explica como criar ferramentas personalizadas para depuração RxJS. Fornece exemplos de implementações práticas de ferramentas de depuração, como rastreamento de stream nomeado, operadores de debug configuráveis e operadores de medição de desempenho.
---

# Ferramentas de Depuração Personalizadas

Criar suas próprias ferramentas de depuração permite uma depuração flexível adaptada aos requisitos do seu projeto.

## Depurando Streams Nomeados

Crie um operador personalizado que pode nomear e rastrear um Observable.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

// Map para gerenciar nomes de stream
const namedStreams = new Map<string, any[]>();

/**
 * Nomear e rastrear um Observable
 */
function tagStream<T>(name: string) {
  if (!namedStreams.has(name)) {
    namedStreams.set(name, []);
  }

  return tap<T>({
    next: value => {
      const log = {
        name,
        type: 'next',
        value,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] next:`, value);
    },
    error: error => {
      const log = {
        name,
        type: 'error',
        error,
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.error(`[${name}] error:`, error);
    },
    complete: () => {
      const log = {
        name,
        type: 'complete',
        timestamp: Date.now()
      };
      namedStreams.get(name)?.push(log);
      console.log(`[${name}] complete`);
    }
  });
}

/**
 * Obter logs para um stream nomeado específico
 */
function getStreamLogs(name: string) {
  return namedStreams.get(name) || [];
}

/**
 * Obter uma lista de todos os streams nomeados
 */
function getAllStreamNames() {
  return Array.from(namedStreams.keys());
}

/**
 * Limpar logs
 */
function clearStreamLogs(name?: string) {
  if (name) {
    namedStreams.set(name, []);
  } else {
    namedStreams.clear();
  }
}
```

### Exemplos de uso

Aqui está um exemplo de nomear e rastrear um Observable.

```ts
import { interval } from 'rxjs';
import { map, take } from 'rxjs';

// Nomear o Observable
interval(1000)
  .pipe(
    tagStream('interval-stream'),
    map(x => x * 2),
    take(5)
  )
  .subscribe();

// Verificar logs após 3 segundos
setTimeout(() => {
  console.log('Todos os streams:', getAllStreamNames());
  console.log('logs do interval-stream:', getStreamLogs('interval-stream'));
}, 3000);

// Saída:
// [interval-stream] next: 0
// [interval-stream] next: 1
// [interval-stream] next: 2
// Todos os streams: ['interval-stream']
// logs do interval-stream: [
//   { name: 'interval-stream', type: 'next', value: 0, timestamp: 1697280000000 },
//   { name: 'interval-stream', type: 'next', value: 1, timestamp: 1697280001000 },
//   { name: 'interval-stream', type: 'next', value: 2, timestamp: 1697280002000 }
// ]
```

### Rastrear múltiplos streams

Nomear e gerenciar múltiplos streams.

```ts
import { interval, fromEvent } from 'rxjs';
import { map, take } from 'rxjs';

// Nomear múltiplos streams
interval(1000)
  .pipe(
    tagStream('timer-stream'),
    map(x => x * 2),
    take(3)
  )
  .subscribe();

const button = document.querySelector('button');
if (button) {
  fromEvent(button, 'click')
    .pipe(
      tagStream('click-stream'),
      take(5)
    )
    .subscribe();
}

// Verificar todos os streams
console.log('Rastreando streams:', getAllStreamNames());
// Saída: ['timer-stream', 'click-stream']
```

> [!NOTE]
> **Sobre rxjs-spy**
>
> `rxjs-spy` era uma biblioteca útil para depurar Observables, mas não é mais mantida e tem problemas de compatibilidade com o RxJS mais recente.
>
> Em vez disso, recomendamos usar operadores de debug personalizados como mostrado acima. Eles são mais flexíveis e podem ser personalizados para os requisitos do seu projeto.

## Criar operador de debug personalizado

Criar seus próprios operadores de debug permite maior flexibilidade de depuração.

```ts
import { Observable } from 'rxjs';
import { tap } from 'rxjs';

interface DebugOptions {
  enabled?: boolean;
  label?: string;
  logValues?: boolean;
  logErrors?: boolean;
  logComplete?: boolean;
  logTimestamp?: boolean;
}

/**
 * Operador de debug personalizado
 */
function debug<T>(options: DebugOptions = {}) {
  const {
    enabled = true,
    label = 'Debug',
    logValues = true,
    logErrors = true,
    logComplete = true,
    logTimestamp = false
  } = options;

  if (!enabled) {
    return (source: Observable<T>) => source;
  }

  return tap<T>({
    next: value => {
      if (logValues) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] next:`, value);
      }
    },
    error: error => {
      if (logErrors) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.error(`${timestamp} [${label}] error:`, error);
      }
    },
    complete: () => {
      if (logComplete) {
        const timestamp = logTimestamp ? `[${new Date().toISOString()}]` : '';
        console.log(`${timestamp} [${label}] complete`);
      }
    }
  });
}

// Exemplo de uso
import { of } from 'rxjs';
import { map } from 'rxjs';

of(1, 2, 3)
  .pipe(
    debug({ label: 'Entrada', logTimestamp: true }),
    map(x => x * 2),
    debug({ label: 'Após Map', logTimestamp: true })
  )
  .subscribe();

// Saída:
// [2025-10-14T12:00:00.000Z] [Entrada] next: 1
// [2025-10-14T12:00:00.001Z] [Após Map] next: 2
// [2025-10-14T12:00:00.001Z] [Entrada] next: 2
// [2025-10-14T12:00:00.002Z] [Após Map] next: 4
// [2025-10-14T12:00:00.002Z] [Entrada] next: 3
// [2025-10-14T12:00:00.003Z] [Após Map] next: 6
// [2025-10-14T12:00:00.003Z] [Entrada] complete
// [2025-10-14T12:00:00.004Z] [Após Map] complete
```

## Operador de debug para medição de desempenho

Um operador de medição de desempenho que registra automaticamente o tempo de execução e o número de valores.

```ts
import { tap } from 'rxjs';

function measure<T>(label: string) {
  let startTime: number;
  let count = 0;

  return tap<T>({
    subscribe: () => {
      startTime = performance.now();
      console.log(`⏱️ [${label}] Início`);
    },
    next: value => {
      count++;
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Valor #${count} (${elapsed.toFixed(2)}ms):`, value);
    },
    complete: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Complete (Total: ${elapsed.toFixed(2)}ms, ${count} valores)`);
    },
    error: () => {
      const elapsed = performance.now() - startTime;
      console.log(`⏱️ [${label}] Erro (${elapsed.toFixed(2)}ms)`);
    }
  });
}

// Exemplo de uso
import { interval } from 'rxjs';
import { take, delay } from 'rxjs';

interval(100)
  .pipe(
    take(5),
    measure('Interval Stream'),
    delay(50)
  )
  .subscribe();

// Saída:
// ⏱️ [Interval Stream] Início
// ⏱️ [Interval Stream] Valor #1 (150.23ms): 0
// ⏱️ [Interval Stream] Valor #2 (250.45ms): 1
// ⏱️ [Interval Stream] Valor #3 (350.67ms): 2
// ⏱️ [Interval Stream] Valor #4 (450.89ms): 3
// ⏱️ [Interval Stream] Valor #5 (551.12ms): 4
// ⏱️ [Interval Stream] Complete (Total: 551.12ms, 5 valores)
```

## Resumo

Ao criar ferramentas de depuração personalizadas:

- ✅ **Streams Nomeados** - identifique e rastreie múltiplos streams por nome
- ✅ **Configuração flexível** - Operador de debug adaptado aos requisitos do projeto
- ✅ **Medição de Desempenho** - Registre automaticamente o tempo de execução e o número de valores
- ✅ **Gerenciamento de Log** - registre e recupere logs com timestamp

É recomendado que essas ferramentas sejam habilitadas apenas no ambiente de desenvolvimento e desabilitadas no ambiente de produção.

## Páginas Relacionadas

- [Estratégias Básicas de Depuração](/pt/guide/debugging/) - Como usar operador tap e ferramentas do desenvolvedor
- [Cenários Comuns de Depuração](/pt/guide/debugging/common-scenarios) - Solução de problemas específicos
- [Depuração de Desempenho](/pt/guide/debugging/performance) - Monitoramento de subscription, verificação de uso de memória
