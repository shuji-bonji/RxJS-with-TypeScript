---
description: Esta página explica como usar schedulers no RxJS e aprender a controlar o processamento assíncrono usando observeOn e subscribeOn. Técnicas práticas para otimização de desempenho e prevenção de bloqueio de UI, como controle de tempo de execução, gerenciamento de contexto de execução e priorização de tarefas, são apresentadas com exemplos de código TypeScript.
---
# Controle de processamento assíncrono

O scheduler no RxJS é um mecanismo importante para controlar o tempo e o contexto de execução do processamento assíncrono. Este capítulo explica como o scheduler é usado para controlar o processamento assíncrono.

## Papel do Scheduler

O scheduler desempenha três papéis importantes:

|Papel|Descrição|
|---|---|
|Controlar tempo de execução|Decidir quando executar tarefas|
|Gerenciar contexto de execução|Determinar em quais threads e ambiente de execução executar tarefas|
|Priorização de tarefas|Gerenciar a ordem de execução de múltiplas tarefas|

## Compreendendo o processamento síncrono e assíncrono

### Comportamento padrão (execução síncrona)

Por padrão, os operadores RxJS são executados de forma tão síncrona quanto possível.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

console.log('Iniciar execução');

of(1, 2, 3)
  .pipe(
    map((x) => {
      console.log(`map: ${x}`);
      return x * 2;
    })
  )
  .subscribe((x) => console.log(`subscribe: ${x}`));

console.log('Finalizar execução');

// Saída:
// Iniciar execução
// map: 1
// subscribe: 2
// map: 2
// subscribe: 4
// map: 3
// subscribe: 6
// Finalizar execução
```

### Assincronização com scheduler

O processamento pode ser assincronizado usando o scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('Iniciar execução');

of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler)
  )
  .subscribe(x => console.log(`subscribe: ${x}`));

console.log('Finalizar execução');

// Saída:
// Iniciar execução
// Finalizar execução
// subscribe: 1
// subscribe: 2
// subscribe: 3
```

## Operadores que usam o scheduler

### Operador observeOn

O operador `observeOn` altera o contexto de execução de um stream. Ele emite valores com o scheduler especificado.

```ts
import { interval, animationFrameScheduler } from 'rxjs';
import { take, observeOn } from 'rxjs';

// Exemplo de uso para animação
interval(16)
  .pipe(
    take(10),
    observeOn(animationFrameScheduler)
  )
  .subscribe(() => {
    // Executar em sincronia com frames de animação
    updateAnimation();
  });

function updateAnimation() {
  // Processamento de atualização de animação
}
```

> [!TIP]
> Para explicações detalhadas, exemplos práticos e precauções sobre o operador `observeOn`, consulte a página do operador [observeOn](../operators/utility/observeOn.md).

### Operador subscribeOn

O operador `subscribeOn` controla quando iniciar a assinatura de um stream.

```ts
import { of, asyncScheduler } from 'rxjs';
import { subscribeOn, tap } from 'rxjs';

console.log('Antes do início da assinatura');

of('Execução de tarefa')
  .pipe(
    tap(() => console.log('Início da tarefa')),
    subscribeOn(asyncScheduler)
  )
  .subscribe(value => console.log(value));

console.log('Após o início da assinatura');

// Saída:
// Antes do início da assinatura
// Após o início da assinatura
// Início da tarefa
// Execução de tarefa
```

> [!TIP]
> Para explicações detalhadas, exemplos práticos e diferenças em relação a `observeOn`, consulte a página do operador [subscribeOn](../operators/utility/subscribeOn.md).

## Exemplos práticos de processamento assíncrono

### Controlando requisições de API

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

// Saída:
// Adicionado à fila: /users
// Adicionado à fila: /posts
// Adicionado à fila: /comments
// Concluído: /users/1 resultado
// Concluído: /posts/1 resultado
// Concluído: /comments/1 resultado
```

### Evitar bloqueio de threads de UI

Utilize o scheduler para evitar bloqueio de threads de UI ao processar grandes quantidades de dados.

```ts
import { from, asapScheduler } from 'rxjs';
import { observeOn, bufferCount } from 'rxjs';

const largeDataSet = Array.from({ length: 10000 }, (_, i) => i);

// Tamanho do lote
const batchSize = 100;
// Calcular número total de lotes
const totalBatches = Math.ceil(largeDataSet.length / batchSize);
// Contador de lotes
let batchIndex = 0;

from(largeDataSet)
  .pipe(
    bufferCount(100), // Agrupar 100 itens por vez
    observeOn(asapScheduler) // O mais rápido possível, mas não bloqueia UI
  )
  .subscribe((batch) => {
    batchIndex++;
    processBatch(batch, batchIndex, totalBatches);
  });

function processBatch(
  batch: number[],
  batchIndex: number,
  totalBatches: number
) {
  // Processar dados do lote
  const processed = batch.map((n) => n * 2);
  console.log(
    `Lote ${batchIndex} de ${totalBatches} concluído: ${processed.length} itens processados.`
  );
}

// Saída:
// Lote 1 de 100 concluído: 100 itens processados.
// Lote 2 de 100 concluído: 100 itens processados.
// ...
// ...
// Lote 100 de 100 concluído: 100 itens processados.
```

## Otimização de desempenho e depuração

### Testando com o Scheduler

O seguinte é um exemplo de teste de processamento assíncrono usando TestScheduler.

```ts
import { TestScheduler } from 'rxjs/testing';
import { delay } from 'rxjs';
import { beforeEach, describe, expect, it } from 'vitest';

describe('Teste de processamento assíncrono', () => {
  let scheduler: TestScheduler;

  beforeEach(() => {
    scheduler = new TestScheduler((actual, expected) => {
      expect(actual).toEqual(expected);
    });
  });

  it('Testar operador delay', () => {
    scheduler.run(({ cold, expectObservable }) => {
      const source = cold('a-b-c|');
      const expected =    '1000ms a-b-(c|)';

      const result = source.pipe(delay(1000, scheduler));

      expectObservable(result).toBe(expected);
    });
  });
});
```

### Saída de log para depuração

O seguinte é um exemplo de saída de log para verificar a operação do scheduler.

```ts
import { of, asyncScheduler } from 'rxjs';
import { tap, observeOn } from 'rxjs';

console.log('Início');

of(1, 2, 3)
  .pipe(
    tap(value => console.log(`[Antes do scheduler - sync] Valor: ${value}`)),
    observeOn(asyncScheduler),  // Usar asyncScheduler
    tap(value => console.log(`[Após o scheduler - async] Valor: ${value}`))
  )
  .subscribe();

console.log('Fim');

// Saída real:
// Início
// [Antes do scheduler - sync] Valor: 1
// [Antes do scheduler - sync] Valor: 2
// [Antes do scheduler - sync] Valor: 3
// Fim
// [Após o scheduler - async] Valor: 1
// [Após o scheduler - async] Valor: 2
// [Após o scheduler - async] Valor: 3
```

Usando `asyncScheduler`, você pode verificar o comportamento assíncrono conforme esperado. Enquanto `queueScheduler` usa uma fila de microtask, que é processada durante a execução de código síncrono, `asyncScheduler` usa setTimeout internamente, então ele roda completamente de forma assíncrona.

## Exemplo mostrando diferenças no comportamento do scheduler

Este exemplo mostra a diferença no tempo de execução de diferentes schedulers.

```ts
import { of, queueScheduler, asyncScheduler, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

console.log('1: Início');

// Processamento síncrono
of('sync').subscribe(value => console.log(`2: ${value}`));

// queueScheduler (microtask)
of('queue')
  .pipe(observeOn(queueScheduler))
  .subscribe(value => console.log(`3: ${value}`));

// asapScheduler (microtask)
of('asap')
  .pipe(observeOn(asapScheduler))
  .subscribe(value => console.log(`4: ${value}`));

// asyncScheduler (macrotask)
of('async')
  .pipe(observeOn(asyncScheduler))
  .subscribe(value => console.log(`5: ${value}`));

Promise.resolve().then(() => console.log('6: Promise'));

console.log('7: Fim');

// Ordem de saída real:
// 1: Início
// 2: sync
// 7: Fim
// 3: queue
// 4: asap
// 6: Promise
// 5: async
```


## Melhores Práticas

1. **Use o scheduler apenas quando necessário**: Se o comportamento síncrono padrão for suficiente, não force o uso do scheduler.

2. **Selecione o scheduler apropriado**: Selecione o melhor scheduler para sua aplicação.
   - Animação: `animationFrameScheduler`
   - Prevenção de bloqueio de UI: `asapScheduler`
   - Processamento de fila: `queueScheduler`
   - Processamento assíncrono: `asyncScheduler`

3. **Monitoramento de desempenho**: monitore constantemente o impacto no desempenho do uso do scheduler

4. **Facilidade de teste**: use `TestScheduler` para escrever testes para processamento assíncrono.

## Erros comuns e contramedidas

### Dessincronização excessiva

Este é um exemplo de como evitar assincronização desnecessária e assincronizar apenas onde necessário.

```ts
// ❌ Assincronização desnecessária
of(1, 2, 3)
  .pipe(
    observeOn(asyncScheduler),
    map(x => x * 2),
    observeOn(asyncScheduler),  // Assincronização duplicada
    filter(x => x > 3)
  )
  .subscribe();

// ✅ Assincronizar apenas onde necessário
of(1, 2, 3)
  .pipe(
    map(x => x * 2),
    filter(x => x > 3),
    observeOn(asyncScheduler)  // Assincronizar tudo de uma vez no final
  )
  .subscribe();
```

### Uso incorreto do scheduler

Esta é uma comparação de uso incorreto e correto.

```ts
// ❌ Uso incorreto
interval(1000)
  .pipe(
    subscribeOn(animationFrameScheduler)  // Não afeta interval
  )
  .subscribe();

// ✅ Uso correto
interval(1000, animationFrameScheduler)  // Especificar scheduler no momento da criação
  .subscribe();
```

## Resumo

O scheduler é uma ferramenta poderosa para controle fino de processamento assíncrono no RxJS. Usado corretamente, pode otimizar o desempenho, evitar bloqueio de threads de UI e facilitar testes. No entanto, é importante usá-lo apenas quando necessário, pois assincronização excessiva pode na verdade piorar o desempenho.

Na próxima seção, discutiremos em detalhes os diferentes tipos de schedulers e como usá-los.
