---
description: "O operador elementAt é um operador de filtragem do RxJS que recupera apenas valores em uma determinada posição de índice. Ele funciona de forma semelhante ao acesso ao índice da matriz."
---

# elementAt - Recuperado por especificação de índice

O operador `elementAt` recupera **apenas o valor na posição de índice especificada** do Observable e completa o fluxo imediatamente. Ele funciona de forma semelhante a `array[index]` de uma matriz.

## Sintaxe básica e uso

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Saída.: 30(Índice2Value)
```

**Fluxo de operação**:.
1. 10 (índice 0) → pular
2. 20 (índice 1) → pular
3. 30 (índice 2) → saída e conclusão
4. 40, 50 não avaliados

[🌐 Documentação oficial do RxJS - elementAt](https://rxjs.dev/api/operators/elementAt)

## 💡 Padrão de utilização típico.

- **Paginação**: obter o primeiro item em uma página específica.
- Obtenção de dados com garantia de ordem**: obtenção do enésimo evento ou mensagem.
- Testes e depuração**: validar o valor de uma posição específica.
- Acesso semelhante a uma matriz**: tratar o Observable como uma matriz

## Exemplo prático de código 1: contagem regressiva de eventos

Este é um exemplo de execução de uma ação no enésimo clique.

```ts
import { fromEvent } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICriar
const output = document.createElement('div');
output.innerHTML = '<h3>5Clique uma vez para exibir a mensagem</h3>';
document.body.appendChild(output);

const button = document.createElement('button');
button.textContent = 'Clique em';
document.body.appendChild(button);

const counter = document.createElement('div');
counter.style.marginTop = '10px';
counter.textContent = 'mais5Clique uma vez';
output.appendChild(counter);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.fontSize = '18px';
result.style.fontWeight = 'bold';
output.appendChild(result);

let clickCount = 0;

// Clique no evento
const clicks$ = fromEvent(button, 'click');

// Para exibição de contagem
clicks$.subscribe(() => {
  clickCount++;
  const remaining = 5 - clickCount;
  if (remaining > 0) {
    counter.textContent = `mais${remaining}Clique uma vez`;
  } else {
    counter.textContent = '';
  }
});

// 5Segunda vez (índice)4Cliques detectados de
clicks$.pipe(
  elementAt(4)
).subscribe(() => {
  result.textContent = '🎉 Alcançados！';
  result.style.color = 'green';
  button.disabled = true;
});
```

- O quinto clique (índice 4) conclui a ação.
- Ele começa em 0, assim como o índice da matriz.

## Exemplo prático de código 2: Obter o número N do fluxo de dados.

Este é um exemplo de recuperação de uma ordem específica de valores de dados publicados em intervalos regulares.

```ts
import { interval } from 'rxjs';
import { elementAt, map } from 'rxjs';

// UICriar
const container = document.createElement('div');
document.body.appendChild(container);

const title = document.createElement('h3');
title.textContent = 'Do fluxo de dadosNObter o segundo';
container.appendChild(title);

const input = document.createElement('input');
input.type = 'number';
input.placeholder = 'Digite o índice (0〜do fluxo de dados (9)';
input.min = '0';
input.max = '9';
input.style.marginRight = '10px';
container.appendChild(input);

const getButton = document.createElement('button');
getButton.textContent = 'Obter';
container.appendChild(getButton);

const status = document.createElement('div');
status.style.marginTop = '10px';
container.appendChild(status);

const result = document.createElement('div');
result.style.marginTop = '10px';
result.style.padding = '10px';
result.style.border = '1px solid #ccc';
result.style.display = 'none';
container.appendChild(result);

// Fluxo de dados (0.5Os valores são emitidos a cada segundo,10até 1)
const data$ = interval(500).pipe(
  map(i => ({ index: i, value: Math.floor(Math.random() * 100), timestamp: Date.now() }))
);

getButton.addEventListener('click', () => {
  const index = parseInt(input.value);

  if (isNaN(index) || index < 0 || index > 9) {
    status.textContent = '0〜do fluxo de dados (9Digite um intervalo de';
    status.style.color = 'red';
    return;
  }

  status.textContent = `Índice ${index} O valor está sendo recuperado...`;
  status.style.color = 'blue';
  result.style.display = 'none';
  getButton.disabled = true;
  input.disabled = true;

  data$.pipe(
    elementAt(index)
  ).subscribe({
    next: data => {
      status.textContent = '';
      result.style.display = 'block';
      result.innerHTML = `
        <strong>✅ Recuperação bem-sucedida</strong><br>
        Índice: ${data.index}<br>
        Valor: ${data.value}<br>
        Registro de data e hora: ${new Date(data.timestamp).toLocaleTimeString()}
      `;
      result.style.color = 'green';
      result.style.backgroundColor = '#e8f5e9';
      getButton.disabled = false;
      input.disabled = false;
    },
    error: err => {
      status.textContent = '';
      result.style.display = 'block';
      result.textContent = `❌ Erro: ${err.message}`;
      result.style.color = 'red';
      result.style.backgroundColor = '#ffebee';
      getButton.disabled = false;
      input.disabled = false;
    }
  });
});
```

- Recupera valores em um índice especificado de um fluxo emitido a cada 0,5 segundo.
- Será gerado um erro se o índice estiver fora do intervalo.

## 🆚 Comparação com operadores semelhantes

### elementAt vs take vs first

```ts
import { from } from 'rxjs';
import { elementAt, take, first, skip } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// elementAt: Somente os valores em um índice específico são recuperados
numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Saída.: 30

// take: Desde o inícioNObter um valor desde o início
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Saída.: 10, 20, 30

// skip + first: elementAt Equivalente a (redundante)
numbers$.pipe(
  skip(2),
  first()
).subscribe(console.log);
// Saída.: 30
```

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

numbers$.pipe(
  elementAt(2)
).subscribe(console.log);
// Saída.: 30(Índice2Value)
---
description: elementAtオペレーターは、指定されたインデックス位置の値のみを取得するRxJSフィルタリングオペレーターです。配列のインデックスアクセスに似た動作をします。
---


## ⚠️ Notas.

### 1. Se o índice estiver fora do intervalo

Se o índice especificado não for alcançado antes da conclusão do fluxo, será gerado um erro.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]); // 3Apenas um

numbers$.pipe(
  elementAt(5) // Índice5Solicitação
).subscribe({
  next: console.log,
  error: err => console.error('Erro:', err.message)
});
// Saída.: Erro: no elements in sequence
```

### 2. especifique os valores padrão.

Para evitar erros, os valores padrão podem ser especificados.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30]);

// Especificar um valor padrão
numbers$.pipe(
  elementAt(5, 999) // Índice5Se não estiver presente, retorna999Retorna um
).subscribe({
  next: console.log,
  error: err => console.error('Erro:', err.message)
});
// Saída.: 999
```

### Use com fluxos assíncronos

Em fluxos assíncronos, aguarde até que a posição do índice seja alcançada.

```ts
import { interval } from 'rxjs';
import { elementAt } from 'rxjs';

// 1Emite um valor a cada segundo
interval(1000).pipe(
  elementAt(3) // Índice3(4(valor do segundo)
).subscribe(console.log);
// 3Saída após segundos: 3
```

### 4. índices negativos não são permitidos

Não é possível especificar índices negativos.

```ts
import { from } from 'rxjs';
import { elementAt } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ❌ Índices negativos são erros
numbers$.pipe(
  elementAt(-1)
).subscribe({
  next: console.log,
  error: err => console.error('Erro:', err.message)
});
// Erro: ArgumentOutOfRangeError: index out of range
```

Use takeLast ou last para chegar ao final do array.

```ts
import { from } from 'rxjs';
import { takeLast, last } from 'rxjs';

const numbers$ = from([10, 20, 30, 40, 50]);

// ✅ Obter o último valor
numbers$.pipe(
  last()
).subscribe(console.log);
// Saída.: 50

// ✅ Obter o últimoNObter o último valor
numbers$.pipe(
  takeLast(2)
).subscribe(console.log);
// Saída.: 40, 50
```

## 📚 Operadores relacionados.

- **[take](. /take)** - N tomado desde o início.
- **[first](. /first)** - obtém o primeiro valor.
- **[last](. /last)** - obtém o último valor
- **[skip](. /skip)** - pula os primeiros N valores
- **[takeLast](. /takeLast)** - obtém os últimos N valores

## Resumo.

O operador elementAt recupera apenas o valor na posição de índice especificada.

- O mesmo comportamento do acesso ao índice da matriz.
- Ideal para recuperar o enésimo valor
- Valores padrão podem ser especificados para evitar erros.
- ⚠️ Erro se o índice estiver fora do intervalo (sem valor padrão)
- ⚠️ Não são permitidos índices negativos
- ⚠️ Os fluxos assíncronos esperam até serem alcançados
