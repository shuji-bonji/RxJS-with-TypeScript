---
description: "O operador findIndex é um operador de filtragem RxJS que retorna o índice do primeiro valor que satisfaz a condição. Se não for encontrado, ele retorna -1."
---

# findIndex - obtém o índice que corresponde à condição

O operador findIndex retorna **o índice do primeiro valor que corresponde à condição** e completa o fluxo imediatamente. Retorna `-1` se nenhum valor for encontrado.

## 🔰 Sintaxe básica e uso

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Saída.: 4(primeiro par8índice do primeiro par)
```

**Fluxo de operação**:.
1. 1 (índice 0) → ímpar, pular
2. 3 (índice 1) → ímpar, pular
3. 5 (índice 2) → Ímpar, pular
4. 7 (índice 3) → Ímpar, pular
5. 8 (índice 4) → número par, saída do índice 4 e conclusão

[🌐 Documentação oficial do RxJS - findIndex](https://rxjs.dev/api/operators/findIndex)

## 💡 Padrão de utilização típico.

- **Posicionamento em uma matriz**: obter a posição de um elemento que satisfaça uma condição específica.
- Verificação da ordem**: quantas vezes um elemento que atende a uma determinada condição aparece
- Reorganização de dados**: processamento usando informações de índice.
- Verificação de existência**: verifica a existência de um elemento, verificando se ele é -1 ou não.

## Exemplo prático de código 1: pesquisa em uma lista de tarefas

Este é um exemplo de como encontrar o local de uma tarefa com condições específicas em uma lista de tarefas.

```ts
import { from, fromEvent } from 'rxjs';
import { findIndex } from 'rxjs';

interface Task {
  id: number;
  title: string;
  priority: 'high' | 'medium' | 'low';
  completed: boolean;
}

const tasks: Task[] = [
  { id: 1, title: 'Resposta de e-mail', priority: 'low', completed: true },
  { id: 2, title: 'Preparação de documentos', priority: 'medium', completed: true },
  { id: 3, title: 'Preparação de reuniões', priority: 'high', completed: false },
  { id: 4, title: 'Revisão de código', priority: 'high', completed: false },
  { id: 5, title: 'Atualização de documentos', priority: 'low', completed: false }
];

// UICriar
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Pesquisa de tarefas';
container.appendChild(title);

// Exibição da lista de tarefas
const taskList = document.createElement('ul');
taskList.style.listStyle = 'none';
taskList.style.padding = '0';
tasks.forEach((task, index) => {
  const li = document.createElement('li');
  li.style.padding = '5px';
  li.style.borderBottom = '1px solid #eee';
  const status = task.completed ? '✅' : '⬜';
  const priorityBadge = task.priority === 'high' ? '🔴' : task.priority === 'medium' ? '🟡' : '🟢';
  li.textContent = `[${index}] ${status} ${priorityBadge} ${task.title}`;
  taskList.appendChild(li);
});
container.appendChild(taskList);

// Botão Pesquisar
const buttonContainer = document.createElement('div');
buttonContainer.style.marginTop = '10px';
container.appendChild(buttonContainer);

const button1 = document.createElement('button');
button1.textContent = 'Pesquisar a primeira tarefa não concluída';
button1.style.marginRight = '10px';
buttonContainer.appendChild(button1);

const button2 = document.createElement('button');
button2.textContent = 'Pesquisar a primeira tarefa de alta prioridade';
buttonContainer.appendChild(button2);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Pesquisar a primeira tarefa não concluída
// NB.: Originalmente, o padrão recomendado era nivelar com switchMap O padrão recomendado é achatar com
// Aqui, a prioridade de legibilidade é dada a subscribe aninhado (no código de produção switchMap recomendado).
fromEvent(button1, 'click').subscribe(() => {
  // Aninhamento subscribe: Originalmente, o padrão recomendado era nivelar com switchMap Recomenda-se achatar com
  from(tasks).pipe(
    findIndex(task => !task.completed)
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Encontrado em</strong><br>
        Posição: Índice ${index}<br>
        Tarefa: ${task.title}<br>
        Prioridade: ${task.priority}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Tarefa inacabada não encontrada';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});

// Pesquisar a primeira tarefa de alta prioridade
// NB.: Originalmente, o padrão recomendado era nivelar com switchMap O padrão recomendado (no código de produção) é nivelar com switchMap recomendado).
fromEvent(button2, 'click').subscribe(() => {
  // Aninhamento subscribe: Originalmente, o padrão recomendado era nivelar com switchMap Recomenda-se achatar com
  from(tasks).pipe(
    findIndex(task => task.priority === 'high')
  ).subscribe(index => {
    result.style.display = 'block';
    if (index !== -1) {
      const task = tasks[index];
      result.innerHTML = `
        <strong>✅ Encontrado em</strong><br>
        Posição: Índice ${index}<br>
        Tarefa: ${task.title}<br>
        Status de conclusão: ${task.completed ? 'Concluída' : 'Não concluída'}
      `;
      result.style.backgroundColor = '#e8f5e9';
      result.style.color = 'green';
    } else {
      result.textContent = '❌ Não foram encontradas tarefas de alta prioridade';
      result.style.backgroundColor = '#fff3e0';
      result.style.color = 'orange';
    }
  });
});
```

- Encontra a posição da primeira tarefa na lista de tarefas que satisfaz a condição.
- Se não for encontrada, `-1` é retornado.

## Exemplo prático de código 2: detecção de localização de dados em tempo real

Esse exemplo detecta a posição do primeiro valor do fluxo que satisfaz a condição.

```ts
import { interval } from 'rxjs';
import { findIndex, map, take } from 'rxjs';

// UICriar
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Pesquisa de dados em tempo real';
container.appendChild(title);

const status = document.createElement('div');
status.style.marginTop = '10px';
status.textContent = '50Pesquisa de posições em que um valor maior ou igual a...';
container.appendChild(status);

const dataDisplay = document.createElement('div');
dataDisplay.style.marginTop = '10px';
dataDisplay.style.padding = '10px';
dataDisplay.style.border = '1px solid #ccc';
dataDisplay.style.maxHeight = '150px';
dataDisplay.style.overflow = 'auto';
container.appendChild(dataDisplay);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.fontWeight = 'bold';
container.appendChild(result);

// Geração de valores aleatórios (0~100)
const data$ = interval(500).pipe(
  take(20),
  map(i => ({ index: i, value: Math.floor(Math.random() * 100) }))
);

// Exibição de dados
data$.subscribe(data => {
  const div = document.createElement('div');
  const highlight = data.value >= 50 ? 'background-color: #fff9c4;' : '';
  div.style.cssText = `padding: 5px; ${highlight}`;
  div.textContent = `[${data.index}] Valor: ${data.value}`;
  dataDisplay.appendChild(div);
  dataDisplay.scrollTop = dataDisplay.scrollHeight;
});

// 50Pesquisar o índice para o primeiro valor de mais de
data$.pipe(
  findIndex(data => data.value >= 50)
).subscribe(index => {
  status.textContent = '';
  if (index !== -1) {
    result.innerHTML = `
      ✅ 50Valor maior ou igual a encontrado<br>
      Posição: Índice ${index}
    `;
    result.style.color = 'green';
  } else {
    result.textContent = '❌ 50Não foram encontrados valores maiores ou iguais a';
    result.style.color = 'orange';
  }
});
```

- Detecta a posição do primeiro valor acima de 50 a partir de valores aleatórios gerados a cada 0,5 segundo.
- O realce é usado para maior clareza visual.

## 🆚 Comparação com operadores semelhantes

### findIndex vs find vs elementAt

```ts
import { from } from 'rxjs';
import { findIndex, find, elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// findIndex: Retorna o índice do primeiro valor que satisfaz a condição
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log);
// Saída.: 2Retorna o índice do primeiro valor que satisfaz a condição30índice do primeiro par)

// find: Retorna o primeiro valor que satisfaz a condição
numbers$.pipe(
  find(n => n > 25)
).subscribe(console.log);
// Saída.: 30

// elementAt: Retorna o valor no índice especificado
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Saída.: 30
```

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 8, 9, 10]);

numbers$.pipe(
  findIndex(n => n % 2 === 0)
).subscribe(console.log);
// Saída.: 4(primeiro par8índice do primeiro par)
```

## 🔄 Comparação com Array.findIndex() do JavaScript

O `findIndex` do RxJS se comporta de forma semelhante ao método de array do JavaScript `Array.prototype.findIndex()`.


```ts
// JavaScript Matriz de
const numbers = [10, 20, 30, 40, 50];
const index1 = numbers.findIndex(n => n > 25);
console.log(index1); // 2

// RxJS (retorna o primeiro valor no índice especificado que satisfaz a condição) Observable
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);
numbers$.pipe(
  findIndex(n => n > 25)
).subscribe(console.log); // 2
```

**Principais diferenças**.
- **Array**: retorna o resultado de forma síncrona e imediata.
- Observable**: assíncrono, espera que os valores fluam do fluxo

## ⚠️ Notas.

### 1. retorna -1 se não for encontrado

Se nenhum valor satisfizer a condição, retorna `-1` em vez de um erro.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const numbers$ = from([1, 3, 5, 7, 9]);

numbers$.pipe(
  findIndex(n => n > 10)
).subscribe(index => {
  if (index === -1) {
    console.log('Não foi encontrado nenhum valor que satisfaça a condição');
  } else {
    console.log(`Índice: ${index}`);
  }
});
// Saída.: Não foi encontrado nenhum valor que satisfaça a condição
```

### 2. completo quando encontrado pela primeira vez.

O fluxo é concluído assim que o primeiro valor que satisfaz a condição é encontrado.

```ts
import { interval } from 'rxjs';
import { findIndex, tap } from 'rxjs';

interval(1000).pipe(
  tap(val => console.log(`Valor: ${val}`)),
  findIndex(n => n >= 3)
).subscribe(index => {
  console.log(`Índice: ${index}`);
});
// Saída.:
// Valor: 0
// Valor: 1
// Valor: 2
// Valor: 3
// Índice: 3
```

### Segurança de tipo em TypeScript

O findIndex sempre retorna o tipo `number`.

```ts
import { Observable, from } from 'rxjs';
import { findIndex } from 'rxjs';

interface User {
  id: number;
  name: string;
  isActive: boolean;
}

function findFirstInactiveUserIndex(
  users$: Observable<User>
): Observable<number> {
  return users$.pipe(
    findIndex(user => !user.isActive)
  );
}

const users$ = from([
  { id: 1, name: 'Alice', isActive: true },
  { id: 2, name: 'Bob', isActive: false },
  { id: 3, name: 'Charlie', isActive: true }
]);

findFirstInactiveUserIndex(users$).subscribe(index => {
  // index é uma matriz de number tipo
  if (index !== -1) {
    console.log(`O primeiro usuário inativo é o índice ${index} é.`);
  }
});
// Saída.: O primeiro usuário inativo é o índice 1 é.
```

### 4. O índice começa em 0

Assim como nos arrays, os índices começam em 0.

```ts
import { from } from 'rxjs';
import { findIndex } from 'rxjs';

const items$ = from(['A', 'B', 'C', 'D']);

items$.pipe(
  findIndex(item => item === 'A')
).subscribe(console.log);
// Saída.: 0(primeiro elemento)
```

## 📚 Operadores relacionados.

- **[find](./find)** - Obtém o primeiro valor que satisfaz a condição.
- **[elementAt](./elementAt)** - Obtém o valor no índice especificado.
- **[first](./first)** - obtém o primeiro valor.
- **[filter](./filter)** - obtém todos os valores que satisfazem a condição

## Resumo.

O operador findIndex retorna o índice do primeiro valor que satisfaz a condição.

- Comportamento semelhante ao `Array.findIndex()` do JavaScript.
- Ideal quando as informações de índice são necessárias
- Retorna `-1` se não for encontrado (não é um erro)
- É concluído imediatamente quando encontrado
- ⚠️ O valor de retorno é sempre do tipo `number` (-1 ou um número inteiro maior ou igual a 0)
- ⚠️ Use find se o valor em si for necessário
