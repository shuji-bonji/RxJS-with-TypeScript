---
description: partition é uma Creation Function do RxJS que divide um Observable em dois Observables com base em condições. É ideal para processamento de bifurcação como sucesso/falha, válido/inválido, etc.
---

# partition - dividir em dois streams com base em condição

`partition` é uma Creation Function que **divide** um Observable em dois Observables com base em uma condição.
Você pode especificar a condição com uma função predicado (predicate) e obter os valores que satisfazem a condição e os valores que não satisfazem a condição como streams separados.

## Sintaxe básica e uso

```ts
import { partition, of } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6);

// Dividir em números pares e ímpares
const [evens$, odds$] = partition(source$, (value) => value % 2 === 0);

evens$.subscribe((value) => console.log('Par:', value));
// Saída: Par: 2, Par: 4, Par: 6

odds$.subscribe((value) => console.log('Ímpar:', value));
// Saída: Ímpar: 1, Ímpar: 3, Ímpar: 5
```

- `partition` retorna um **array contendo dois Observables**.
- `[0]`: um stream de valores que satisfazem a condição.
- `[1]`: um stream de valores que não satisfazem a condição.

[🌐 Documentação Oficial RxJS - `partition`](https://rxjs.dev/api/index/function/partition)

## Padrões típicos de utilização

- **Dividir processamento de sucesso/falha** (classificação por código de status HTTP)
- **Classificação de eventos** (clique esquerdo/clique direito)
- **Classificação de dados** (válido/inválido, adulto/criança, etc.)
- **Divisão de stream com base em condições**.

## Exemplos de código prático (com UI)

Quando um botão é clicado, o processamento é ramificado dependendo se as coordenadas do clique são para a metade esquerda ou direita da tela.

```ts
import { partition, fromEvent } from 'rxjs';
import { map } from 'rxjs';

// Criar área de saída
const leftArea = document.createElement('div');
leftArea.innerHTML = '<h3>Clique Esquerdo</h3><ul id="left-list"></ul>';
leftArea.style.float = 'left';
leftArea.style.width = '45%';
leftArea.style.padding = '10px';
leftArea.style.background = '#e3f2fd';
document.body.appendChild(leftArea);

const rightArea = document.createElement('div');
rightArea.innerHTML = '<h3>Clique Direito</h3><ul id="right-list"></ul>';
rightArea.style.float = 'right';
rightArea.style.width = '45%';
rightArea.style.padding = '10px';
rightArea.style.background = '#fce4ec';
document.body.appendChild(rightArea);

// Eventos de clique
const clicks$ = fromEvent<MouseEvent>(document, 'click');

// Coordenada X central da tela
const centerX = window.innerWidth / 2;

// Dividir em metades esquerda e direita
const [leftClicks$, rightClicks$] = partition(
  clicks$,
  (event) => event.clientX < centerX
);

// Processar cliques esquerdos
leftClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const leftList = document.getElementById('left-list')!;
  const li = document.createElement('li');
  li.textContent = `Posição: (${pos.x}, ${pos.y})`;
  leftList.appendChild(li);
});

// Processar cliques direitos
rightClicks$.pipe(
  map(event => ({ x: event.clientX, y: event.clientY }))
).subscribe((pos) => {
  const rightList = document.getElementById('right-list')!;
  const li = document.createElement('li');
  li.textContent = `Posição: (${pos.x}, ${pos.y})`;
  rightList.appendChild(li);
});
```

- Clicar na tela será registrado nas listas esquerda e direita de acordo com a posição do clique.
- Dois streams independentes podem ser criados a partir de uma única fonte.

## Exemplo prático: Processamento de ramificação de respostas de API

Exemplo de divisão de sucesso e falha por código de status HTTP

```ts
import { partition, from, of } from 'rxjs';
import { mergeMap, map, catchError, share } from 'rxjs';

interface ApiResponse {
  status: number;
  data?: any;
  error?: string;
}

// Chamadas de API simuladas
const apiCalls$ = from([
  fetch('/api/users/1'),
  fetch('/api/users/999'), // Usuário inexistente
  fetch('/api/users/2'),
]);

// Processar Response e converter para ApiResponse
const responses$ = apiCalls$.pipe(
  mergeMap(fetchPromise => from(fetchPromise)),
  mergeMap(response =>
    from(response.json()).pipe(
      map(data => ({
        status: response.status,
        data: response.ok ? data : undefined,
        error: response.ok ? undefined : (data.message || 'Erro')
      } as ApiResponse)),
      catchError(err => of({
        status: response.status,
        data: undefined,
        error: err.message || 'Falha ao analisar resposta'
      } as ApiResponse))
    )
  ),
  share() // Lidar com 2 assinaturas do partition
);

// Dividir em sucesso (200s) e falha (outros)
const [success$, failure$] = partition(
  responses$,
  (response: ApiResponse) => response.status >= 200 && response.status < 300
);

// Lidar com respostas de sucesso
success$.subscribe((response) => {
  console.log('✅ Sucesso:', response.data);
  // Exibir dados de sucesso na UI
});

// Lidar com respostas de falha
failure$.subscribe((response) => {
  console.error('❌ Falha:', response.error);
  // Exibir mensagem de erro
});
```

## Comparação com filter

### Diferenças básicas

| Método | Descrição | Saída | Caso de Uso |
|--------|-------------|--------|----------|
| `partition` | Dividir uma fonte em dois streams | 2 Observables | Quando você deseja usar ambos os streams **simultaneamente** |
| `filter` | Apenas passa valores que atendem à condição | 1 Observable | Quando apenas um stream é necessário |

### Exemplos de uso

**Use partition para processar ambos os streams simultaneamente**

```ts
import { partition, interval } from 'rxjs';
import { map, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Sucesso</h4><ul id="success-list"></ul>';
successArea.style.float = 'left';
successArea.style.width = '45%';
output.appendChild(successArea);

const failureArea = document.createElement('div');
failureArea.innerHTML = '<h4 style="color: red;">❌ Falha</h4><ul id="failure-list"></ul>';
failureArea.style.float = 'right';
failureArea.style.width = '45%';
output.appendChild(failureArea);

// Stream aleatório de sucesso/falha
const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Tarefa ${i + 1}`
  }))
);

// ✅ partition - lidar com sucesso e falha simultaneamente
const [success$, failure$] = partition(tasks$, task => task.success);

success$.subscribe(task => {
  const successList = document.getElementById('success-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  successList.appendChild(li);
});

failure$.subscribe(task => {
  const failureList = document.getElementById('failure-list')!;
  const li = document.createElement('li');
  li.textContent = task.message;
  failureList.appendChild(li);
});
```

**Use filter se apenas um stream for necessário**

```ts
import { interval } from 'rxjs';
import { map, take, filter } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

const successArea = document.createElement('div');
successArea.innerHTML = '<h4 style="color: green;">✅ Exibir apenas sucesso</h4><ul id="success-only"></ul>';
output.appendChild(successArea);

const tasks$ = interval(1000).pipe(
  take(10),
  map(i => ({
    id: i + 1,
    success: Math.random() > 0.5,
    message: `Tarefa ${i + 1}`
  }))
);

// ✅ filter - processar apenas sucesso (ignorar falhas)
tasks$
  .pipe(filter(task => task.success))
  .subscribe(task => {
    const successList = document.getElementById('success-only')!;
    const li = document.createElement('li');
    li.textContent = task.message;
    successList.appendChild(li);
  });
```

**Use filter duas vezes vs. partition**

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';
import { partition } from 'rxjs';

const numbers$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

// ❌ Usar filter duas vezes - a fonte pode ser executada duas vezes
const evens1$ = numbers$.pipe(filter(n => n % 2 === 0));
const odds1$ = numbers$.pipe(filter(n => n % 2 !== 0));

evens1$.subscribe(n => console.log('Par:', n));
odds1$.subscribe(n => console.log('Ímpar:', n));
// Problema: se numbers$ for um observable cold, será executado duas vezes

// ✅ Usar partition - criar ambos os streams em uma execução
const [evens2$, odds2$] = partition(numbers$, n => n % 2 === 0);

evens2$.subscribe(n => console.log('Par:', n));
odds2$.subscribe(n => console.log('Ímpar:', n));
// Vantagem: criar eficientemente dois streams a partir de uma fonte
```

**Use filter se você quiser ramificar no pipeline**

```ts
import { from } from 'rxjs';
import { filter, map } from 'rxjs';

interface User {
  id: number;
  name: string;
  age: number;
  isActive: boolean;
}

const users$ = from([
  { id: 1, name: 'Alice', age: 25, isActive: true },
  { id: 2, name: 'Bob', age: 30, isActive: false },
  { id: 3, name: 'Carol', age: 35, isActive: true }
]);

// ❌ partition é uma Creation Function, então não pode ser usada em um pipeline
// users$.pipe(
//   map(user => user.name),
//   partition(name => name.startsWith('A')) // Erro
// );

// ✅ Usar filter - disponível em pipeline
users$
  .pipe(
    filter(user => user.isActive),  // Apenas usuários ativos
    map(user => user.name)           // Extrair nome
  )
  .subscribe(console.log);
// Saída: Alice, Carol
```

### Resumo

| Situação | Método Recomendado | Razão |
|-----------|-------------------|--------|
| Deseja processar **ambos** sucesso e falha | `partition` | Pode criar dois streams em uma execução de fonte |
| Deseja processar **apenas** sucesso | `filter` | Simples e claro |
| Deseja ramificar condições no pipeline | `filter` | `partition` não pode ser usado pois é uma Creation Function |
| Deseja ramificar em 3 ou mais com condições complexas | `groupBy` | Pode dividir em vários grupos |

## Observações

### 1. Assinar ambos os streams

Os dois Observables criados em um `partition` **compartilham** a fonte original.
Se você não assinar ambos, o stream original pode não ser totalmente processado.

```ts
const [success$, failure$] = partition(source$, predicate);

// Assinar ambos
success$.subscribe(handleSuccess);
failure$.subscribe(handleFailure);
```

### 2. A fonte é executada duas vezes

O `partition` internamente assina a fonte original duas vezes.
Esteja ciente de quaisquer efeitos colaterais.

```ts
let callCount = 0;
const source$ = new Observable(observer => {
  callCount++;
  console.log(`Contagem de assinatura: ${callCount}`);
  observer.next(1);
  observer.complete();
});

const [a$, b$] = partition(source$, n => n > 0);
a$.subscribe(); // Contagem de assinatura: 1
b$.subscribe(); // Contagem de assinatura: 2
```

Para evitar efeitos colaterais, use `share()`.

```ts
import { share } from 'rxjs';

const shared$ = source$.pipe(share());
const [a$, b$] = partition(shared$, n => n > 0);
```

### 3. Não fornecido como Pipeable Operator

Desde o RxJS 7, `partition` é fornecido como **Creation Function apenas**.
Não pode ser usado dentro de um pipeline.

```ts
// ❌ Não é possível
source$.pipe(
  partition(n => n % 2 === 0) // Erro
);

// ✅ Uso correto
const [evens$, odds$] = partition(source$, n => n % 2 === 0);
```

## Padrões Alternativos

Se você quiser ramificar dentro de um pipeline, use `filter`.

```ts
const source$ = of(1, 2, 3, 4, 5, 6);

const evens$ = source$.pipe(filter(n => n % 2 === 0));
const odds$ = source$.pipe(filter(n => n % 2 !== 0));

// Ou compartilhe a fonte com share
const shared$ = source$.pipe(share());
const evens$ = shared$.pipe(filter(n => n % 2 === 0));
const odds$ = shared$.pipe(filter(n => n % 2 !== 0));
```

## Operadores relacionados

- [`filter`](../../operators/filtering/filter.md) - passa apenas valores que satisfazem uma condição
- [`groupBy`](../../operators/transformation/groupBy.md) - Dividir em vários grupos
- [`share`](../../operators/multicasting/share.md) - Compartilhar uma fonte

## Resumo

`partition` é uma ferramenta poderosa para dividir um Observable em dois com base em uma condição.

- ✅ Ideal para processamento de divisão de sucesso/falha
- ✅ Cria dois streams independentes
- ⚠️ As fontes são assinadas duas vezes (observe os efeitos colaterais)
- ⚠️ Não oferecido como Pipeable Operator
