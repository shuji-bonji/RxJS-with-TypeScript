---
description: "O operador filter classifica os valores em um fluxo com base em uma função condicional especificada, permitindo a passagem apenas dos valores que satisfazem a condição. Ele pode ser utilizado como uma função Type Guard (predicado de tipo) no TypeScript, e também explica a diferença entre ele e o buffer e as ressalvas de transformar uma função de predicado em uma função pura. Esta seção também explica a diferença entre buffers e funções puras."
---

# filter - somente passar valores que correspondam às condições

O operador `filter` classifica os valores em um fluxo com base em uma função condicional especificada e só permite a passagem de valores que atendam à condição.

## 🔰 Sintaxe básica e uso

```ts
import { from } from 'rxjs';
import { filter } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5, 6, 7, 8, 9, 10]);

numbers$.pipe(
  filter(n => n % 2 === 0)
).subscribe(console.log);
// Resultados: 2, 4, 6, 8, 10
```

- Somente os valores que correspondem à condição são transmitidos.
- Funciona de forma semelhante a `Array.prototype.filter()` em arrays, mas é sequencial no Observable.

[🌐 Documentação oficial do RxJS - `filter`](https://rxjs.dev/api/operators/filter)

## 💡 Padrão de utilização típico.

- Validação de valores de entrada de formulário
- Permitir apenas dados de um tipo ou estrutura específica
- Filtragem de eventos de sensores e dados de fluxo

## Exemplos práticos de código (com UI)

Somente listar em tempo real se o número inserido for par.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

const title = document.createElement('h3');
title.innerHTML = 'filter Exemplos práticos de:';
document.body.appendChild(title);

// Criação de campos de entrada
const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Entrada de valores numéricos';
input.style.marginBottom = '10px';
document.body.appendChild(input);

// Criar área de saída
const output = document.createElement('div');
document.body.appendChild(output);

// Fluxo de eventos de entrada
fromEvent(input, 'input')
  .pipe(
    map((e) => parseInt((e.target as HTMLInputElement).value, 10)),
    filter((n) => !isNaN(n) && n % 2 === 0)
  )
  .subscribe((evenNumber) => {
    const item = document.createElement('div');
    item.textContent = `Detecção de números pares: ${evenNumber}`;
    output.prepend(item);
  });

```

- Somente será exibido na saída se o número for par.
- Entradas ímpares ou inválidas são ignoradas.

> [!WARNING] 本番コードでの注意

> A amostra acima omite o cancelamento da assinatura fromEvent para simplificar a explicação. No código real, use `takeUntil(destroy$)`, `take(N)` ou `Subscription.unsubscribe()` para gerenciar explicitamente o ciclo de vida. Mais informações: [Superando dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)

## 🔍 Diferenças com o buffer

| Operador | Operação | Saída. |
|---|---|---|
| filter. | Descarta valores que não **correspondem** à condição | Valores individuais `T`. |
| buffer. | Armazena** valores em uma matriz**. | Matriz `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Somente os valores que correspondem às condições são transmitidos
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Resultados: filter: 0
  // Resultados: filter: 2
  // Resultados: filter: 4
});

// buffer - Armazena valores como uma matriz
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Resultados: buffer: [0, 1]
  // Resultados: buffer: [2, 3, 4]
});
```

## ⚠️ Notas.

### 1. As funções de predicado devem ser funções puras.

As funções de predicado com efeitos colaterais podem causar um comportamento inesperado quando o fluxo é reinscrito.

```ts
// ❌ Exemplo ruim: Efeitos colaterais Sim
let counter = 0;
source$.pipe(
  filter(x => {
    counter++; // Efeito colateral
    return x > 10;
  })
).subscribe();

// ✅ Bom exemplo: Função pura
source$.pipe(
  filter(x => x > 10)
).subscribe();
```

### Use como função de proteção de tipo

Você pode escrevê-la para retornar um predicado de tipo TypeScript (`x is T`) para restringir o tipo depois de passar o filtro.

```ts
import { Observable, of, filter } from 'rxjs';

interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Usada como função de proteção de tipo
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email Não é uma função de proteção de tipo string É inferida como um tipo
});
```

> [!TIP] 型ガードの効果

> Ao retornar o predicado de tipo `user is User & { email: string }`, `user` após `filter` torna `email` uma propriedade obrigatória. Chamadas como `user.email.toLowerCase()` podem ser escritas sem erros de tipo.

## 📚 Operadores relacionados.

- [take](/pt/guide/operators/filtering/take) - somente os primeiros N valores são tomados.
- [first](/pt/guide/operators/filtering/first) - obtém apenas o primeiro valor (também pode ser condicional)
- [distinct](/pt/guide/operators/filtering/distinct) - exclui valores duplicados
- distinctUntilChanged](/pt/guide/operators/filtering/distinctUntilChanged) - exclui o mesmo que o último valor

## Resumo.

O operador `filter` é a ferramenta de filtragem mais básica do RxJS.

- Somente os valores que correspondem às condições são transmitidos.
- Pode ser usado da mesma forma que `.filter()` para arrays.
- Também pode ser usado como uma proteção de tipo TypeScript.
- ⚠️ As funções de predicado devem ser funções puras.
- ⚠️ Nome semelhante, mas uso diferente de buffer (valores individuais vs. arrays)
