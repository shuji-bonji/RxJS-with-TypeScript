---
description: Este curso explica a classificação de tarefas (síncrona, microtask e macrotask) e sua relação com cada scheduler no RxJS desde o básico, incluindo como funciona o event loop do JavaScript, a diferença na ordem de execução e a implementação e operação de setTimeout, Promise, queueMicrotask, etc., e adquira conhecimento que pode ser usado para selecionar um scheduler para RxJS.
---

# Conhecimento Básico de Tarefas e Schedulers

## O que é Processamento Síncrono?
O processamento síncrono é executado imediatamente na ordem em que o código é escrito e não prossegue para o próximo processo até que o processo anterior seja concluído.

#### Exemplo
```ts
console.log('A');
console.log('B');
console.log('C');

// Saída:
// A
// B
// C
```


## O que é Processamento Assíncrono?
O processamento assíncrono é um processamento que não é executado imediatamente, mas é executado após a conclusão do processamento síncrono atual.
O processamento assíncrono inclui "macro tasks" e "micro tasks".


## Macro Task
- Uma tarefa que é executada no próximo ciclo do event loop.
- Exemplos: `setTimeout`, `setInterval`, eventos do navegador

#### Exemplo de Execução
```ts
console.log('Início');
setTimeout(() => console.log('Macro Task'), 0);
console.log('Fim');

// Saída:
// Início
// Fim
// Macro Task
```

### Suporte RxJS
- `asyncScheduler`
  - Usa `setTimeout` internamente
  - Funciona como uma macro task

```ts
import { of, asyncScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Olá')
  .pipe(observeOn(asyncScheduler))
  .subscribe(console.log);

// Saída:
// Olá
```


## Micro Task
- Uma tarefa que é executada imediatamente após a conclusão da tarefa atual, mas antes do início da próxima tarefa.
- Exemplos: `Promise.then`, `queueMicrotask`

#### Exemplo de Execução
```ts
console.log('Início');
Promise.resolve().then(() => console.log('Micro Task'));
console.log('Fim');

// Saída:
// Início
// Fim
// Micro Task
```

### Suporte RxJS
- `asapScheduler`
  - Usa `Promise.resolve().then()` internamente
  - Funciona como uma microtask

```ts
import { of, asapScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Oi')
  .pipe(observeOn(asapScheduler))
  .subscribe(console.log);

// Saída:
// Oi
```


## Tarefa Síncrona
- Código normal a ser executado imediatamente.

### Suporte RxJS
- `queueScheduler`
  - Parece ser síncrono, mas permite controle fino através de enfileiramento de tarefas.

```ts
import { of, queueScheduler } from 'rxjs';
import { observeOn } from 'rxjs';

of('Agora')
  .pipe(observeOn(queueScheduler))
  .subscribe(console.log);

// Saída:
// Agora
```


## Resumo da Ordem de Execução

#### Exemplo de Código
```ts
console.log('1');

setTimeout(() => console.log('2 (setTimeout)'), 0);
Promise.resolve().then(() => console.log('3 (Promise)'));

console.log('4');

// Saída:
// 1
// 4
// 3 (Promise) 👈 Microtask
// 2 (setTimeout) 👈 Macrotask
```


## Tabela de Correspondência de Tarefas e Schedulers RxJS

| Tipo         | Exemplo                       | Scheduler RxJS     |
|--------------|-------------------------------|---------------------|
| Síncrono     | Código normal                 | `queueScheduler`    |
| Microtask    | Promise.then, queueMicrotask  | `asapScheduler`     |
| Macrotask    | setTimeout, setInterval       | `asyncScheduler`    |
