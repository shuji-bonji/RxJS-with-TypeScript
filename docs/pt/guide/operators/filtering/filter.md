---
description: O operador filter é um operador básico do RxJS que filtra valores com base em condições especificadas e é usado para controlar fluxos de dados. Como Array.prototype.filter(), ele usa uma função predicado para determinar quais valores passar, permitindo seleção condicional de valores e filtragem type-safe.
---

# filter - Filtrar Valores com Base em Condições

O operador `filter` passa apenas valores que **satisfazem uma condição especificada** (função predicado).
Este é o mesmo conceito do `Array.prototype.filter()` do JavaScript aplicado a Observables.

## 🔰 Sintaxe Básica e Uso

```ts
import { of } from 'rxjs';
import { filter } from 'rxjs';

const source$ = of(1, 2, 3, 4, 5, 6, 7, 8, 9, 10);

source$.pipe(
  filter(x => x % 2 === 0) // Passa apenas números pares
).subscribe(value => {
  console.log('Valor:', value);
});

// Saída:
// Valor: 2
// Valor: 4
// Valor: 6
// Valor: 8
// Valor: 10
```

- A função predicado `(value) => boolean` determina quais valores passar.
- Apenas valores que retornam `true` são passados para o próximo operador.

## 💡 Padrões de Uso Típicos

- **Filtragem de dados**: Seleciona apenas valores que satisfazem condições específicas
- **Validação de entrada**: Permite apenas valores válidos
- **Processamento condicional**: Processa diferentes streams com base em condições específicas
- **Type guard**: Restringe tipos TypeScript

## 🧠 Exemplo de Código Prático: Validação de Entrada do Usuário

Este exemplo filtra valores de entrada para permitir apenas caracteres numéricos.

```ts
import { fromEvent } from 'rxjs';
import { map, filter } from 'rxjs';

// Criar campo de entrada
const input = document.createElement('input');
input.type = 'text';
input.placeholder = 'Digite apenas números...';
input.style.padding = '8px';
input.style.margin = '10px';
document.body.appendChild(input);

// Área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
output.style.padding = '10px';
output.style.border = '1px solid #ccc';
document.body.appendChild(output);

// Evento de entrada
const input$ = fromEvent<InputEvent>(input, 'input').pipe(
  map(event => (event.target as HTMLInputElement).value),
  filter(value => /^\d*$/.test(value)) // Permite apenas caracteres numéricos
);

input$.subscribe(value => {
  output.textContent = `Valor válido: ${value}`;
  console.log('Valor numérico:', value);
});

// Se caracteres não numéricos forem inseridos, o evento é filtrado
```

## 🔍 Diferença em relação ao buffer

| Operador | Comportamento | Saída |
|:---|:---|:---|
| `filter` | Descarta valores que **não correspondem** à condição | Valor individual `T` |
| `buffer` | **Acumula** valores em um array | Array `T[]` |

```ts
import { interval } from 'rxjs';
import { filter, buffer, take } from 'rxjs';

const source$ = interval(1000).pipe(take(5)); // 0, 1, 2, 3, 4

// filter - Passa apenas valores que correspondem à condição
source$.pipe(
  filter(x => x % 2 === 0)
).subscribe(x => {
  console.log('filter:', x);
  // Saída: filter: 0
  // Saída: filter: 2
  // Saída: filter: 4
});

// buffer - Acumula valores como um array
source$.pipe(
  buffer(interval(2500))
).subscribe(arr => {
  console.log('buffer:', arr);
  // Saída: buffer: [0, 1]
  // Saída: buffer: [2, 3, 4]
});
```

[🌐 Documentação Oficial do RxJS - `filter`](https://rxjs.dev/api/operators/filter)

## ⚠️ Observações

### 1. Funções Predicado Devem Ser Funções Puras

Evite funções predicado com efeitos colaterais.

```ts
// ❌ Exemplo ruim: Com efeitos colaterais
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

### 2. Usando Funções Type Guard

Você pode aproveitar a segurança de tipos do TypeScript.

```ts
interface User {
  id: number;
  name: string;
  email?: string;
}

const users$: Observable<User> = of(
  { id: 1, name: 'Alice', email: 'alice@example.com' },
  { id: 2, name: 'Bob' }
);

// Usar como função type guard
users$.pipe(
  filter((user): user is User & { email: string } => user.email !== undefined)
).subscribe(user => {
  console.log(user.email.toLowerCase()); // email é inferido como tipo string
});
```

## 📚 Operadores Relacionados

- [take](/pt/guide/operators/filtering/take) - Obtém apenas os primeiros N valores
- [first](/pt/guide/operators/filtering/first) - Obtém apenas o primeiro valor (condicionalmente possível)
- [distinct](/pt/guide/operators/filtering/distinct) - Exclui valores duplicados
- [distinctUntilChanged](/pt/guide/operators/filtering/distinctUntilChanged) - Exclui valores que são iguais ao anterior

## Resumo

O operador `filter` é a ferramenta de filtragem mais básica no RxJS.

- ✅ Passa apenas valores que correspondem à condição
- ✅ Pode ser usado da mesma forma que `.filter()` para arrays
- ✅ Pode ser usado como type guard do TypeScript
- ⚠️ Funções predicado devem ser funções puras

> [!WARNING] Atenção em código de produção
> O exemplo acima omite o cancelamento da inscrição de `fromEvent` para simplificar a explicação. Em código real, gerencie explicitamente o ciclo de vida com `takeUntil(destroy$)`, `take(N)`, ou `Subscription.unsubscribe()`. Detalhes: [Superar dificuldades: gerenciamento do ciclo de vida](/pt/guide/overcoming-difficulties/lifecycle-management.md)
