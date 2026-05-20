---
description: "O find é um operador de filtragem RxJS que encontra o primeiro valor que satisfaz uma condição e o produz, completando o fluxo imediatamente. É ideal para situações em que se deseja localizar um elemento específico de uma matriz ou lista, como pesquisar usuários, verificar o inventário ou detectar logs de erros. Se nenhum valor for encontrado, a saída será indefinida e, no TypeScript, o valor de retorno será do tipo T | undefined."
---

# Find - find o primeiro valor que satisfaz a condição

O operador `find` encontra e gera o **primeiro valor que satisfaz a condição** e completa o fluxo imediatamente. Se nenhum valor for encontrado, ele produzirá `undefined`.

## Sintaxe básica e uso

```ts
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  find(n => n % 2 === 0)
).subscribe(console.log);
// Saída.: 8(primeiro número par)
```

**Fluxo de operação**:.
1. verificar 1, 3, 5, 7 → condição não atendida
2. verificar 8 → condição satisfeita → saída 8 e completa
3. 9, 10 não avaliados

[🌐 Documentação oficial do RxJS - `find`](https://rxjs.dev/api/operators/find)

## 🆚 Contraste com o first

O `find` e o `first` são semelhantes, mas seu uso é diferente.

```ts
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// first: Primeiro valor que satisfaz a condição (a condição é opcional)
numbers$.pipe(
  first(n => n > 5)
).subscribe(console.log);
// Saída.: 7

// find: Primeiro valor que satisfaz a condição (a condição é obrigatória)
numbers$.pipe(
  find(n => n > 5)
).subscribe(console.log);
// Saída.: 7
```

| Operador. | Especificação da condição | Se nenhum valor for encontrado | Caso de uso. |
|---|---|---|---|
| first()` | Opção | Erro (`EmptyError`) | Obter o primeiro valor |
| `first(predicate)` | Opcional | Erro (`EmptyError`) | Obtenção condicional. |
| find(predicate)` | Obrigatório. | Saída `undefined`. | Pesquisa e verificação de existência |

## 💡 Padrão de utilização típico

1. **Pesquisa do usuário**.

```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface User {
     id: number;
     name: string;
     email: string;
   }

   const users$ = from([
     { id: 1, name: 'Alice', email: 'alice@example.com' },
     { id: 2, name: 'Bob', email: 'bob@example.com' },
     { id: 3, name: 'Charlie', email: 'charlie@example.com' }
   ] as User[]);

   // ID(a condição é opcional)2Pesquisar usuários com
   users$.pipe(
     find(user => user.id === 2)
   ).subscribe(user => {
     if (user) {
       console.log(`Encontrado: ${user.name}`);
     } else {
       console.log('Usuário não encontrado');
     }
   });
   // Saída.: Encontrado: Bob
   ```

2. **Verificação de inventário**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface Product {
     id: string;
     name: string;
     stock: number;
   }

   const products$ = from([
     { id: 'A1', name: 'NotebookPC', stock: 0 },
     { id: 'A2', name: 'Mouse', stock: 15 },
     { id: 'A3', name: 'Teclados', stock: 8 }
   ] as Product[]);

   // Descubra o que está fora de estoque
   products$.pipe(
     find(product => product.stock === 0)
   ).subscribe(product => {
     if (product) {
       console.log(`Fora de estoque: ${product.name}`);
     } else {
       console.log('Todos em estoque');
     }
   });
   // Saída.: Fora de estoque: NotebookPC
   ```

3. **Pesquisar registro de erros**
   ```ts
   import { from } from 'rxjs';
   import { find } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 4, level: 'info' as const, message: 'Retry successful' }
   ] as LogEntry[]);

   // Pesquisar o primeiro erro
   logs$.pipe(
     find(log => log.level === 'error')
   ).subscribe(log => {
     if (log) {
       console.log(`Detecção de erros: ${log.message} (Tempo: ${log.timestamp})`);
     }
   });
   // Saída.: Detecção de erros: Connection failed (Tempo: 3)
   ```

## 🧠 Exemplo prático de código (pesquisa de produtos)

Este é um exemplo de pesquisa de produtos que correspondem a critérios específicos do estoque.

```

ts.
import { from, fromEvent } from 'rxjs';
import { find } from 'rxjs';

interface Product {
  id: string;
  name: string;
  price: número;
  category: string;
}

const products: Product[] = [
  { id: 'P1', name: 'Wireless mouse', price: 2980, category: 'PC peripherals' }
  { id: 'P2', name: 'Mechanical Keyboard', price: 8980, category: 'PC Peripherals' }
  { id: 'P3', name: 'USB memory stick 64GB', price: 1480, category: 'Storage' }
  { id: 'P4', name: 'Monitor 27-inch', price: 29800, category: 'Displays' }
  { id: 'P5', name: 'laptop stand', price: 3980, category: 'PC peripherals' }
];

// Criação de elementos da interface do usuário
const container = document.createElement('div');.
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Product Search';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Digite o preço máximo';
input.style.marginRight = '10px';
contêiner.appendChild(input);

const searchButton = document.createElement('button');
searchButton.textContent = 'search';
contêiner.appendChild(searchButton);

const result = document.createElement('div');
result.style.marginTop = '10px';
contêiner.appendChild(result);

// Processamento de pesquisa
// Observação: originalmente o padrão recomendado é achatar com um switchMap, mas,
// Observação: embora o padrão recomendado seja nivelar com um switchMap, // aqui aninhamos o subscribe para facilitar a leitura, // porque ele inclui a validação da IU (retorno antecipado).
// Considere uma implementação plana usando switchMap no código de produção.
fromEvent(searchButton, 'click').subscribe(() => {
  const maxPrice = parseInt(input.value);.

  se (isNaN(maxPrice)) {
    result.textContent = 'Por favor, digite um preço';
    result.style.colour = 'red';
    return;
  }

  // Subscribe: originalmente recomendado para nivelar com switchMap
  from(products).pipe(
    find(product => product.price <= maxPrice)
  ).subscribe(product => {
    if (product) {
      result.innerHTML = `
        <strong>Encontrado! </strong><br>
        Nome do produto: ${product.name}<br>
        Preço: ${product.price.toLocaleString()}<br>
        Categoria: ${product.category}
      `;
      result.style.color = 'green';
    } else {
      result.textContent = `¥${maxPrice.toLocaleString()} or less product not found `;
      result.style.color = 'range'; }
    }
  });
});

```

Esse código pesquisa e exibe o primeiro produto abaixo do preço inserido pelo usuário.

## 🎯 filter A diferença entre

`find` e `filter` são usados para finalidades diferentes.

```

ts.
import { from } from 'rxjs';
import { find, filter } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

// filter: gera todos os valores que correspondem à condição
numbers$.pipe(
  filter(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('filter complete')
});
// Saída: 7, 8, 9, 10, filter complete

// find: gera apenas o primeiro valor que corresponde à condição
numbers$.pipe(
  find(n => n > 5)
).subscribe({
  next: console.log,.
  complete: () => console.log('find complete')
});
// saída: 7, find complete

```

| Operador | Número de saídas | Tempo de conclusão | Caso de uso |
|---|---|---|---|
| `filter(predicate)` | Todos os valores que correspondem à condição | Na conclusão do fluxo original | Refinamento de dados |
| `find(predicate)` | Somente o primeiro valor que corresponde aos critérios | Imediatamente após a descoberta | Pesquisa e verificação de existência |

## 📋 Uso com segurança de tipo

TypeScript Este é um exemplo de implementação à prova de tipos que utiliza genéricos em

```

ts.
import { Observable, from } from 'rxjs';
import { find } from 'rxjs';

interface Task {
  id: número;
  title: string;;
  completed: booleano;
  prioridade: 'alta' | 'média' | 'baixa'; }
}

function findTaskById(
  tasks$: Observable,.
  id: number
): Observable | undefined> {
  return tasks$.pipe(
    find(task => task.id === id)
  );
}

function findFirstIncompleteTask(
  tasks$: Observable
): Observable | undefined> {
  return tasks$.pipe(
    find(task => !task.completed)
  );
}

// Exemplo de uso
const tasks$ = from([.
  { id: 1, title: 'Task A', completed: true, priority: 'high' as const }
  { id: 2, title: 'Task B', completed: false, priority: 'medium' as const }
  { id: 3, title: 'Task C', completed: false, priority: 'low' as const }
] as Task[]);.

// Pesquisar por ID
findTaskById(tasks$, 2).subscribe(task => {
  if (task) {
    console.log(`found: ${task.title}`);
  } else {
    console.log('Tarefa não encontrada'); }
  }
});
// Saída: encontrada: tarefa B

// Localizar tarefas não concluídas
findFirstIncompleteTask(tasks$).subscribe(task => {
  if (task) {
    console.log(`Próxima tarefa: ${task.title} (prioridade: ${task.priority})`);
  }
});
// Saída: próxima tarefa: tarefa B (prioridade: média)

```

## 🔄 find e findIndex A diferença entre

RxJSnos operadores `findIndex` também estão disponíveis.

```

ts
import { from } from 'rxjs';
import { find, findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// find: retorna um valor
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);.
// saída: 30

// findIndex: retorna o índice
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);.
// Saída: 2 (índice de 30)

```

| Operador | Retorna o valor | se o valor não for encontrado |
|---|---|---|
| `find(predicate)` | O próprio valor | `undefined` |
| `findIndex(predicate)` | Índice (valor numérico) | `-1` |

## ⚠️ Erros comuns

> [!NOTE]
> `find` se o valor não for encontrado. `undefined` é emitido. Isso não resulta em um erro. Se for necessário um erro, use `first` para ser usado.

### Error (Erro).: Tratamento de erro esperado se o valor não for encontrado.

```

ts.
import { from } from 'rxjs';
import { find } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ❌ Exemplo ruim: tratamento de erros esperado, mas não chamado
numbers$.pipe(
  find(n => n > 10)
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err) // não chamado
});
// saída: indefinido

```

### Positivo: undefined Verificar ou first use o

```

ts.
import { from } from 'rxjs';
import { find, first } from 'rxjs';

const numbers$ = from([1, 3, 5, 7]);

// ✅ Bom exemplo 1: verificação de indefinição
numbers$.pipe(
  find(n => n > 10)
).subscribe(result => {
  if (result ! == undefined) {
    console.log('Found:', result);
  } else {
    console.log('Não encontrado:'); }
  }
});
// Saída: não encontrado

// ✅ Bom exemplo 2: use o primeiro se precisar de um erro
numbers$.pipe(
  first(n => n > 10, 0) // especifique o valor padrão
).subscribe({
  next: console.log,.
  error: err => console.log('Error:', err.message)
});
// Saída: 0
```

## 🎓 Resumo

### Quando você deve usar find.
- Quando você quiser encontrar o primeiro valor que satisfaça uma condição
- Quando você quiser verificar a existência de um valor
- Quando você quiser tratar um valor como "indefinido" se ele não for encontrado.
- Quando você quiser encontrar um elemento específico em uma matriz ou lista

### Quando você deve usar first
- Se você quiser obter o primeiro valor
- Se você quiser emitir um erro se o valor não for encontrado

### Quando o filter deve ser usado?
- Se você precisar de todos os valores que correspondem a uma condição
- Se você quiser filtrar os dados

### Notas.
- ⚠️ `find` gera `undefined` se não for encontrado (não é um erro)
- ⚠️ Conclui imediatamente com o primeiro valor que satisfaz a condição
- ⚠️ O TypeScript fornece um valor de retorno do tipo `T | undefined`.

## Próxima etapa.

- **[first](. /first)** - saiba como obter o primeiro valor.
- **[filter](. /filter)** - saiba como filtrar com base em condições.
- **[findIndex](https://rxjs.dev/api/operators/findIndex)** - saiba como obter o índice do primeiro valor que satisfaz uma condição (documentação oficial)
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - aprenda casos de uso reais
