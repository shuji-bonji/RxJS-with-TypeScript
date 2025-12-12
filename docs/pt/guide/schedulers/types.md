---
description: Esta página explica em detalhes as características, implementações e aplicações dos principais schedulers no RxJS, como asyncScheduler e queueScheduler. Compreenda as diferenças entre macro tasks, micro tasks e processamento síncrono, e aprenda o tempo de execução e características de cada scheduler. Ao usá-los adequadamente, você pode otimizar o desempenho e comportamento da sua aplicação.
---

# Tipos de Schedulers e Como Usá-los

O RxJS fornece múltiplos schedulers para diferentes aplicações. Cada scheduler tem seu próprio tempo de execução e características específicas, e o uso apropriado de cada um pode otimizar o desempenho e comportamento da sua aplicação.

## Classificação de Schedulers

Os schedulers RxJS se dividem em três categorias principais.

1. **Macro Task**: executado na próxima fila de tarefas no event loop
2. **Micro-task**: executado imediatamente após a conclusão da tarefa atual e antes do início da próxima tarefa
3. **Processamento síncrono**: execução imediata

Para mais informações, consulte [Conhecimento Básico de Tarefas e Schedulers](./task-and-scheduler-basics.md) para detalhes.

## Principais schedulers

### asyncScheduler

#### Características
- **Implementação interna**: usa setTimeout
- **Tempo de execução**: macro tasks
- **Uso**: Processamento assíncrono geral, processamento com intervalo de tempo

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

of('Processamento assíncrono')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fim');

// Saída:
// 1: Início
// 2: Fim
// 3: Processamento assíncrono
```

#### Casos de Uso

Este exemplo simula um processo de computação pesada.

```ts
import { asyncScheduler, map, observeOn, of } from "rxjs";

function heavyComputation(value: number): number {
  // Simular computação pesada
  let result = value;
  for (let i = 0; i < 1000000; i++) {
    result = Math.sin(result);
  }
  return result;
}

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(value => heavyComputation(value))
  )
  .subscribe(result => {
    console.log(`Resultado do cálculo: ${result}`);
  });
```

### queueScheduler

#### Características
- **Implementação interna**: fila de micro task
- **Tempo de execução**: dentro da tarefa atual (parece síncrono)
- **Uso**: Enfileiramento de tarefas, otimização de recursão

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

of('Processamento de fila')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fim');

// Saída:
// 1: Início
// 2: Processamento de fila
// 3: Fim
```

#### Casos de Uso

Este é um exemplo de otimização de um processo recursivo.

```ts
import { Observable, of, queueScheduler } from 'rxjs';
import { observeOn, expand, take, map } from 'rxjs';

// Otimização de processamento recursivo
function fibonacci(n: number): Observable<number> {
  return of([0, 1]).pipe(
    observeOn(queueScheduler),
    expand(([a, b]) => of([b, a + b])),
    map(([a]) => a),
    take(n)
  );
}

fibonacci(10).subscribe(value => console.log(value));
```

### asapScheduler

#### Características
- **Implementação interna**: Promise.resolve().then() ou setImmediate
- **Tempo de execução**: microtasks
- **Uso**: Para execução assíncrona o mais rápido possível

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

of('Processamento ASAP')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fim');

// Saída:
// 1: Início
// 2: Fim
// 3: Processamento ASAP
```

#### Casos de Uso

Este é um exemplo de otimização de eventos de movimento do mouse.

```ts
import { fromEvent, asapScheduler } from 'rxjs';
import { observeOn, map } from 'rxjs';

// Otimização de eventos de movimento do mouse
fromEvent(document, 'mousemove')
  .pipe(
    observeOn(asapScheduler),
    map(event => ({
      x: (event as MouseEvent).clientX,
      y: (event as MouseEvent).clientY
    }))
  )
  .subscribe(position => {
    // Processamento de atualização de UI
    updateCursor(position);
  });
```

### animationFrameScheduler

#### Características
- **Implementação interna**: requestAnimationFrame
- **Tempo de execução**: antes da próxima renderização da tela
- **Uso**: Animação, processo de desenho para 60fps

#### Exemplo de uma animação de rotação simples

Este é um exemplo de rotação de um elemento circular em HTML.

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// Criar elemento HTML
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Configurações de animação
let rotation = 0;

// Animar a 60fps por 2 segundos
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2 segundos = 120 frames
    map(() => {
      rotation += 3;  // Rotacionar 3 graus por frame
      return rotation;
    })
  )
  .subscribe(angle => {
    // Realmente rotacionar o elemento DOM
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Por que animationFrameScheduler?

O `animationFrameScheduler` executa sincronizadamente com o ciclo de desenho do navegador, o que oferece as seguintes vantagens:

1. **Animação Suave**: Como o processamento é realizado em sincronia com o tempo de renderização do navegador (tipicamente 60 fps), uma animação suave sem cortes pode ser alcançada.
2. **Uso eficiente de recursos**: Quando o navegador desativa a aba, a execução de requestAnimationFrame é automaticamente pausada para evitar uso desnecessário de CPU.
3. **Anti-flickering**: Garante que a computação seja concluída antes da tela ser desenhada, prevenindo flickering e exibição de frames incompletos.

O seguinte é uma comparação de `setInterval` e `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ Animação ineficiente usando setInterval
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // aprox. 60fps

// Problemas:
// - Não sincronizado com o tempo de renderização do navegador
// - Continua a executar mesmo em abas de fundo
// - Incapaz de garantir 60fps precisos

// ✅ Animação eficiente usando animationFrameScheduler
interval(0, animationFrameScheduler)
  .pipe(
    map(() => {
      position += 1;
      return position;
    })
  )
  .subscribe(pos => {
    element.style.transform = `translateX(${pos}px)`;
  });

// Benefícios
// - Sincroniza com o tempo de renderização do navegador
// - Pausa automaticamente em abas de fundo
// - Alcança 60fps estável
```


#### Exemplo de animação que segue o mouse

Criar uma animação de círculo que segue o cursor do mouse.

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Criar um círculo que segue
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Deixar eventos do mouse passar
document.body.appendChild(circle);

// Posições atuais e de destino
let currentX = 0;
let currentY = 0;
let targetX = 0;
let targetY = 0;

// Monitorar eventos de movimento do mouse
const mouseMove$ = fromEvent<MouseEvent>(document, 'mousemove')
  .pipe(
    map(event => ({
      x: event.clientX,
      y: event.clientY
    }))
  );

// Loop de animação
interval(0, animationFrameScheduler)
  .pipe(
    withLatestFrom(mouseMove$),
    map(([_, mousePos]) => mousePos)
  )
  .subscribe(({ x, y }) => {
    // Definir posição do mouse como destino
    targetX = x;
    targetY = y;

    // Mover gradualmente da posição atual para a posição de destino (easing)
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;

    // Atualizar elemento DOM
    circle.style.left = `${currentX - 15}px`;  // Ajustar para posição central
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guia de uso do scheduler

### Comparação por tempo de execução

O seguinte é um exemplo comparando a ordem de execução de cada scheduler.

```ts
import { of, asyncScheduler, queueScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

// Processamento síncrono
of('sync').subscribe(v => console.log(`2: ${v}`));

// queueScheduler (microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(v => console.log(`3: ${v}`));

// asapScheduler (microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(v => console.log(`4: ${v}`));

// asyncScheduler (macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(v => console.log(`5: ${v}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fim');

// Ordem de execução:
// 1: Início
// 2: sync
// 7: Fim
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```

### Critérios de Seleção por Uso

O seguinte é um resumo das características e aplicações adequadas de cada scheduler.

| Scheduler | Características | Usos Adequados |
|--------------|------|----------|
| asyncScheduler | Usa setTimeout, totalmente assíncrono | Processamento demorado, execução atrasada |
| queueScheduler | Síncrono mas otimiza recursão | Processamento recursivo, gerenciamento de fila de tarefas |
| asapScheduler | Execução assíncrona o mais rápido possível | Tratamento de eventos, processamento de resposta rápida |
| animationFrameScheduler | Sincronizado com renderização de tela | Animação, atualizações de UI, desenvolvimento de jogos |

## Casos de uso prático

### Processamento de grandes quantidades de dados

Este é um exemplo de enfileiramento de requisições e processamento em ordem.

```ts
import { from, queueScheduler } from 'rxjs';
import { mergeMap, observeOn, tap } from 'rxjs';

interface ApiRequest {
  endpoint: string;
  id: number;
}

const requests: ApiRequest[] = [
  { endpoint: '/users', id: 1 },
  { endpoint: '/posts', id: 1 },
  { endpoint: '/comments', id: 1 },
];

// Enfileirar requisições e processar em ordem
from(requests)
  .pipe(
    observeOn(queueScheduler),
    tap((req) => console.log(`Adicionado à fila: ${req.endpoint}`)),
    mergeMap(
      (req) =>
        // Simular requisição de API real
        new Promise((resolve) => {
          setTimeout(() => {
            resolve(`${req.endpoint}/${req.id} resultado`);
          }, 1000);
        })
    )
  )
  .subscribe((result) => console.log(`Concluído: ${result}`));
```

### Tratamento de mensagens WebSocket

Este é um exemplo de processamento de mensagens WebSocket que requer uma resposta rápida.

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// Nota: Este é pseudo-código para ilustrar o conceito
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // Tratar como string
});

socket$
  .pipe(
    // Processamento de mensagem que requer resposta rápida
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('Mensagem recebida:', msg);
}
```

### Controle de repetição de erro

Ao utilizar o scheduler com o operador `retry`, o tempo de repetição pode ser finamente controlado.

#### Controle básico de repetição

A opção `delay` do operador `retry` usa internamente o `asyncScheduler` para controlar o intervalo de repetição.

```ts
import { throwError, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

// Simulação de chamada de API
function fetchData(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.7) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Erro de rede'));
    })
  );
}

fetchData(1)
  .pipe(
    retry({
      count: 3,
      delay: 1000  // Aguardar 1 segundo com asyncScheduler antes de repetir
    })
  )
  .subscribe({
    next: result => console.log('✅ Sucesso:', result),
    error: error => console.log('❌ Erro final:', error.message)
  });
```

#### Utilização de scheduler em exponential back-off

Para controle mais avançado, o backoff exponencial pode ser implementado combinando `retryWhen` e `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Erro temporário'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;

          // Verificar contagem máxima de repetição
          if (retryCount > 3) {
            console.log('❌ Contagem máxima de repetição atingida');
            throw error;
          }

          // Backoff exponencial: 1 segundo, 2 segundos, 4 segundos...
          const delayTime = Math.pow(2, index) * 1000;
          console.log(`🔄 Repetir ${retryCount} vezes (após ${delayTime}ms)`);

          // timer usa internamente asyncScheduler
          return timer(delayTime);
        })
      )
    )
  )
  .subscribe({
    next: result => console.log('✅ Sucesso:', result),
    error: error => console.log('❌ Erro final:', error.message)
  });

// Saída de exemplo:
// 🔄 Repetir 1 vezes (após 1000ms)
// 🔄 Repetir 2 vezes (após 2000ms)
// 🔄 Repetir 3 vezes (após 4000ms)
// ❌ Contagem máxima de repetição atingida
// ❌ Erro final: Erro temporário
```

#### Quando asyncScheduler é explicitamente especificado

Especificar explicitamente um scheduler específico permite controle mais flexível, como substituí-lo por `TestScheduler` durante testes.

```ts
import { throwError, asyncScheduler, of } from 'rxjs';
import { retryWhen, mergeMap, delay } from 'rxjs';

function fetchDataWithScheduler(id: number, scheduler = asyncScheduler) {
  return of(id).pipe(
    mergeMap(() => throwError(() => new Error('Erro'))),
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          if (index >= 2) throw error;

          // Especificar explicitamente scheduler
          return of(null).pipe(
            delay(1000, scheduler)
          );
        })
      )
    )
  );
}

// Ambiente de produção: usar asyncScheduler
fetchDataWithScheduler(1).subscribe({
  error: err => console.log('Erro:', err.message)
});

// Ambiente de teste: pode ser substituído por TestScheduler
```

> [!TIP]
> Para padrões de implementação detalhados e métodos de depuração para processamento de repetição, consulte a página [retry e catchError](/pt/guide/error-handling/retry-catch).
> - Uso detalhado do operador retry
> - Padrões de combinação com catchError
> - Técnicas de depuração de repetição (rastreamento do número de tentativas, logging, etc.)

## Impacto no Desempenho

### Overhead do scheduler

Este é um exemplo de como evitar uso excessivo do scheduler e otimizar para processamento em lote.

```ts
import { range, asyncScheduler, pipe } from 'rxjs';
import { bufferCount, map, observeOn, tap } from 'rxjs';

// ❌ Uso excessivo de scheduler
range(1, 1000)
  .pipe(
    observeOn(asyncScheduler),  // 1000 setTimeouts
    map(x => x * 2),
    // tap(console.log)
  )
  .subscribe();

// ✅ Otimizar com processamento em lote
range(1, 1000)
  .pipe(
    bufferCount(100),
    observeOn(asyncScheduler),  // 10 setTimeouts
    map(batch => batch.map(x => x * 2)),
    // tap(console.log)
  )
  .subscribe();
```

## Resumo

A escolha do scheduler tem um impacto significativo no desempenho e capacidade de resposta da aplicação. Compreender as características de cada scheduler e usá-los em situações apropriadas garantirá operação eficiente e suave. Como diretriz geral,

- Para processamento assíncrono geral, use `asyncScheduler`
- `queueScheduler` para processamento recursivo e enfileiramento síncrono
- `asapScheduler` para tempos de resposta rápidos
- `animationFrameScheduler` para animação

são recomendados.
