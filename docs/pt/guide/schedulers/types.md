---
description: "Saiba mais sobre as características, a implementação e o uso dos principais schedulers do RxJS, como asyncScheduler e queueScheduler. Entenda as diferenças entre macrotarefas, microtarefas e processamento síncrono e conheça o tempo de execução e as características de cada agendador. Você pode otimizar o desempenho e o comportamento do seu aplicativo usando-os adequadamente."
---

# Tipos de programadores e como usá-los

O RxJS fornece vários agendadores para diferentes aplicativos. Cada agendador tem seu próprio tempo de execução e características específicas e pode ser usado adequadamente para otimizar o desempenho e o comportamento do aplicativo.

## Classificação dos agendadores

Os agendadores do RxJS se enquadram em três categorias principais.

1. **Macro tarefas**: executadas na próxima fila de tarefas no loop de eventos
2. **Micro-tarefa**: executada imediatamente após a conclusão da tarefa atual e antes do início da próxima tarefa
3. **Processamento síncrono**: execução imediata

Para obter mais informações, consulte [Task and Scheduler Basics](. /task-and-scheduler-basics.md).

## Agendadores principais.

### Scheduler

#### Recursos.
- Implementação interna**: usa setTimeout.
- Tempo de execução**: tarefas macro
- Uso**: processamento assíncrono geral, processamento de lapso de tempo

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

#### Caso de uso

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
    console.log(`Resultados do cálculo: ${result}`);
  });
```

### Scheduler

#### Recursos.
- Implementação interna**: fila de microtarefas
- Tempo de execução**: dentro da tarefa atual (parece síncrono)
- Usos**: enfileiramento de tarefas, otimização de recursão

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

of('Processamento em fila')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`2: ${value}`));

console.log('3: Fim');

// Saída:
// 1: Início
// 2: Processamento em fila
// 3: Fim
```

#### Caso de uso

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

### Scheduler

#### Recursos.
- Implementação interna**: Promise.resolve().then() ou setImmediate
- Tempo de execução**: microtarefas
- Uso**: para execução assíncrona o mais rápido possível

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

of('ASAPProcessamento')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`3: ${value}`));

console.log('2: Fim');

// Saída:
// 1: Início
// 2: Fim
// 3: ASAPProcessamento
```

#### Caso de uso

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
    // UIProcessamento de atualização de
    updateCursor(position);
  });
```

### Scheduler do quadro de animação

#### Recursos.
- Implementação interna**: requestAnimationFrame
- Tempo de execução**: antes da próxima renderização da tela
- Uso**: Animação, processo de desenho para 60fps.

#### Exemplo de uma animação de rotação simples

```ts
import { animationFrameScheduler, interval } from 'rxjs';
import { take, map } from 'rxjs';

// HTMLCriação de elementos
const box = document.createElement('div');
box.style.width = '100px';
box.style.height = '100px';
box.style.backgroundColor = 'blue';
box.style.position = 'absolute';
box.style.top = '100px';
box.style.left = '100px';
document.body.appendChild(box);

// Configuração de animações
let rotation = 0;

// 60fpsNos2Segundos de animação
interval(0, animationFrameScheduler)
  .pipe(
    take(120),  // 60fps × 2Segundos = 120Quadro
    map(() => {
      rotation += 3;  // 1Cada quadro3Grau de rotação
      return rotation;
    })
  )
  .subscribe(angle => {
    // DOMRotação real do elemento
    box.style.transform = `rotate(${angle}deg)`;
  });
```

#### Por que precisamos do Scheduler?

O Scheduler executa suas operações de forma síncrona com o ciclo de desenho do navegador, o que tem as seguintes vantagens.

1.**Animações suaves**: como o processamento é sincronizado com o tempo de renderização do navegador (normalmente 60 fps), é possível obter animações suaves e sem cortes.
2.**Uso eficiente de recursos**: quando o navegador desativa a guia, a execução do requestAnimationFrame é automaticamente pausada, evitando o uso desnecessário da CPU.
3, **Evitar a cintilação da tela**: a cintilação da tela e os quadros incompletos são evitados porque o cálculo é concluído de forma confiável antes de a tela ser desenhada.

Aqui está uma comparação entre `setInterval` e `animationFrameScheduler`.

```ts
import { animationFrameScheduler, interval, map } from "rxjs";

// ❌ setIntervalAnimação ineficiente usando
let position = 0;
const intervalId = setInterval(() => {
  position += 1;
  element.style.transform = `translateX(${position}px)`;
}, 16);  // aproximadamente60fps

// Problemas:
// - Não sincronizado com os tempos de renderização do navegador
// - Continua a ser executada mesmo em guias em segundo plano
// - Precisão60fpsnão pode ser garantida

// ✅ animationFrameSchedulerUso eficiente de animação
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

// Vantagens
// - Sincronizado com os tempos de desenho do navegador
// - Pausa automática nas guias em segundo plano
// - Estável60fpsPermite
```

#### Exemplo de animação que segue o mouse

```ts
import { fromEvent, animationFrameScheduler, interval } from 'rxjs';
import { withLatestFrom, observeOn, map } from 'rxjs';

// Criação de um círculo a ser seguido
const circle = document.createElement('div');
circle.style.width = '30px';
circle.style.height = '30px';
circle.style.borderRadius = '50%';
circle.style.backgroundColor = 'red';
circle.style.position = 'fixed';
circle.style.pointerEvents = 'none';  // Transparente aos eventos do mouse
document.body.appendChild(circle);

// Posição atual e de destino
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
    // Definir a posição do mouse como alvo
    targetX = x;
    targetY = y;
    
    // Movimento gradual da posição atual para a posição de destino々Movimento gradual (atenuação) da posição atual para a posição de destino
    currentX += (targetX - currentX) * 0.1;
    currentY += (targetY - currentY) * 0.1;
    
    // DOMAtualizar elemento
    circle.style.left = `${currentX - 15}px`;  // Ajuste para a posição central
    circle.style.top = `${currentY - 15}px`;
  });
```

## Guia para usar diferentes programadores

### Comparação por tempo de execução

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

### Critérios de seleção por aplicativo


## Casos práticos de uso

### Processamento de grandes quantidades de dados


### Processamento de mensagens WebSocket

```ts
import { webSocket } from 'rxjs/webSocket';
import { asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

// 注: これは概念を示す疑似コードです
const socket$ = webSocket<any>({
  url: 'wss://your-websocket-server.com',
  deserializer: msg => msg.data // 文字列として扱う
});

socket$
  .pipe(
    // 高速な応答が必要なメッセージ処理
    observeOn(asapScheduler)
  )
  .subscribe(message => {
    handleMessage(message);
  });

function handleMessage(msg: any) {
  console.log('メッセージ受信:', msg);
}
```

### Controle de repetição de erro

Ao utilizar o agendador com o operador `retry`, o tempo das tentativas pode ser controlado com precisão.

#### Controle básico de novas tentativas

A opção `delay` do operador `retry` usa o `asyncScheduler` internamente para controlar o intervalo de retry.


```

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

#### Utilização do Scheduler com back-off exponencial

Como um controle mais avançado, o backoff exponencial pode ser implementado combinando o `retryWhen` e o `asyncScheduler`.

```ts
import { throwError, timer, of } from 'rxjs';
import { retry, mergeMap } from 'rxjs';

function fetchDataWithBackoff(id: number) {
  return of(id).pipe(
    mergeMap(() => {
      const random = Math.random();
      if (random > 0.9) {
        return of({ id, data: 'success' });
      }
      return throwError(() => new Error('Temporary error'));
    })
  );
}

fetchDataWithBackoff(1)
  .pipe(
    // RxJS 7.3+ 推奨: retry({ count, delay }) 形式
    retry({
      count: 3, // Máx.3回まで再試行
      delay: (error, retryCount) => {
        // 指数バックオフ: 1秒, 2秒, 4秒...
        const delayTime = Math.pow(2, retryCount - 1) * 1000;
        console.log(`🔄 リトライ ${retryCount}回目 (${delayTime}ms後)`);

        // timer は内部的に asyncScheduler を使用
        return timer(delayTime);
      }
    })
  )
  .subscribe({
    next: result => console.log('✅ 成功:', result),
    error: error => {
      console.log('❌ Máx.リトライ数に到達');
      console.log('❌ 最終Erro:', error.message);
    }
  });

// SaídaEx.:
// 🔄 リトライ 1回目 (1000ms後)
// 🔄 リトライ 2回目 (2000ms後)
// 🔄 リトライ 3回目 (4000ms後)
// ❌ Máx.リトライ数に到達
// ❌ 最終Erro: Temporary error
```

#### Ao especificar explicitamente um Scheduler

A especificação explícita de um Scheduler específico permite um controle mais flexível, como, por exemplo, substituí-lo pelo Scheduler durante o teste.


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


> - Os padrões de implementação detalhados e os métodos de depuração para o tratamento de retry estão descritos na página [retry e catchError](/pt/guide/error-handling/retry-catch).
> - Uso detalhado do operador retry.
> - Padrões de combinação com catchError.
> - Técnicas de depuração para tentativas (por exemplo, tentativas de rastreamento, registro)

## Impacto no desempenho.

### Sobrecarga do escalonador


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

## Resumo.

A escolha do agendador tem um impacto significativo sobre o desempenho e a capacidade de resposta de um aplicativo. Compreender as características de cada agendador e usá-los nas situações certas garantirá uma operação eficiente e tranquila. Como diretriz geral,

- Scheduler para processamento assíncrono geral.
- Scheduler para processamento recursivo e enfileiramento síncrono
- Scheduler para tempos de resposta rápidos
- Scheduler para animação

para animação.