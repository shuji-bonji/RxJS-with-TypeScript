---
description: window é um operador RxJS que divide um Observable fonte no momento em que outro Observable emite valores, ideal para processamento avançado de stream orientado a eventos.
titleTemplate: ':title | RxJS'
---

# window - Divide Observable no Momento de Outro Observable

O operador `window` agrupa os valores de um Observable fonte **até que outro Observable emita valores** e gera esse grupo como um **novo Observable**.
Enquanto `buffer` retorna um array, `window` retorna um **Observable&lt;T&gt;**, permitindo que operadores adicionais sejam aplicados a cada janela.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

// Emite valores a cada 100ms
const source$ = interval(100);

// Usa evento de clique como gatilho
const clicks$ = fromEvent(document, 'click');

source$.pipe(
  window(clicks$),
  mergeAll() // Achata cada janela
).subscribe(value => {
  console.log('Valor na janela:', value);
});

// Uma nova janela começa com cada clique
```

- Cada vez que `clicks$` emite um valor, uma nova janela (Observable) é criada.
- Cada janela pode ser tratada como um Observable independente.

[🌐 Documentação Oficial RxJS - `window`](https://rxjs.dev/api/operators/window)

## 💡 Padrões Típicos de Uso

- Particionamento de stream orientado a eventos
- Aplicar processamento diferente a cada janela
- Agrupamento de dados com delimitação dinâmica
- Processamento agregado para cada janela

## 🔍 Diferença de buffer

| Operador | Saída | Caso de Uso |
|:---|:---|:---|
| `buffer` | **Array (T[])** | Processar valores agrupados juntos |
| `window` | **Observable&lt;T&gt;** | Processamento de stream diferente para cada grupo |

```ts
import { interval, timer } from 'rxjs';
import { buffer, window, mergeAll } from 'rxjs';

const source$ = interval(100);
const trigger$ = timer(1000, 1000);

// buffer - Gera como array
source$.pipe(
  buffer(trigger$)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Saída: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// window - Gera como Observable
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor na janela:', value);
  });
});
```

## 🧠 Exemplo de Código Prático 1: Contagem Por Janela

Este exemplo dispara no clique do botão e conta o número de eventos até esse ponto.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, map, mergeAll, scan } from 'rxjs';

// Criar botão
const button = document.createElement('button');
button.textContent = 'Delimitar Janela';
document.body.appendChild(button);

// Área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Emite valores a cada 100ms
const source$ = interval(100);

// Gatilho ao clicar no botão
const clicks$ = fromEvent(button, 'click');

let windowCount = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const currentWindow = ++windowCount;
    console.log(`Janela ${currentWindow} iniciada`);

    // Conta valores em cada janela
    return window$.pipe(
      scan((count) => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  output.textContent = `Janela atual: ${windowCount}, Contagem: ${count}`;
});
```

- Cada vez que um botão é clicado, uma nova janela é criada.
- O número de valores em cada janela é contado em tempo real.

## 🎯 Exemplo de Código Prático 2: Processamento Diferente Para Cada Janela

Este é um exemplo avançado que aplica processamento diferente a cada janela.

```ts
import { interval, fromEvent } from 'rxjs';
import { window, take, mergeAll, map } from 'rxjs';

const source$ = interval(200);
const clicks$ = fromEvent(document, 'click');

let windowNumber = 0;

source$.pipe(
  window(clicks$),
  map(window$ => {
    const current = ++windowNumber;

    if (current % 2 === 0) {
      // Janelas pares: Obter apenas os primeiros 3 itens
      console.log(`Janela ${current}: Obter primeiros 3 itens`);
      return window$.pipe(take(3));
    } else {
      // Janelas ímpares: Obter todos
      console.log(`Janela ${current}: Obter todos`);
      return window$;
    }
  }),
  mergeAll()
).subscribe(value => {
  console.log(`Valor: ${value} (Janela ${windowNumber})`);
});
```

- Você pode aplicar condicionalmente processamento diferente para cada janela.
- Cada janela é um Observable independente, então você pode combinar operadores livremente.

## 🎯 Exemplo Prático: Controle com Múltiplos Gatilhos

```ts
import { interval, merge, fromEvent, timer } from 'rxjs';
import { window, mergeAll, scan, map } from 'rxjs';

const source$ = interval(100);

// Múltiplos gatilhos: clique ou 3 segundos decorridos
const clicks$ = fromEvent(document, 'click');
const threeSeconds$ = timer(3000, 3000);
const trigger$ = merge(clicks$, threeSeconds$);

source$.pipe(
  window(trigger$),
  map((window$, index) => {
    console.log(`Janela ${index + 1} iniciada`);

    // Calcula soma para cada janela
    return window$.pipe(
      scan((sum, value) => sum + value, 0)
    );
  }),
  mergeAll()
).subscribe(sum => {
  console.log('Soma atual:', sum);
});
```

## ⚠️ Notas

### 1. Gerenciamento de Inscrição de Janela

Cada janela é um Observable independente, então deve ser explicitamente inscrita.

```ts
source$.pipe(
  window(trigger$)
).subscribe(window$ => {
  // Os valores não fluirão a menos que você se inscreva na própria janela
  window$.subscribe(value => {
    console.log('Valor:', value);
  });
});
```

Ou use `mergeAll()`, `concatAll()`, `switchAll()`, etc. para achatar.

```ts
source$.pipe(
  window(trigger$),
  mergeAll() // Mescla todas as janelas
).subscribe(value => {
  console.log('Valor:', value);
});
```

### 2. Cuidado com Vazamentos de Memória

**Problema**: Se o Observable de gatilho não emitir valores, a primeira janela permanece aberta para sempre e os valores acumulam infinitamente.

#### ❌ Exemplo Ruim: Gatilho Não Ocorre

```ts
import { interval, fromEvent } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100); // Continua a emitir valores a cada 100ms

// Botão não existe, ou usuário não clica
const button = document.querySelector('#start-button'); // Possivelmente null
const clicks$ = fromEvent(button, 'click'); // Erro ou nunca dispara

source$.pipe(
  window(clicks$), // Primeira janela não fecha se clicks$ não disparar
  mergeAll()
).subscribe();

// Problemas:
// - Se clicks$ não emitir, a primeira janela fica aberta
// - Valores de source$ (0, 1, 2, 3...) continuam a acumular na memória
// - Causa vazamento de memória
```

#### ✅ Bom Exemplo 1: Definir Timeout

Defina um timeout para evitar que a primeira janela fique aberta por muito tempo.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = button ? fromEvent(button, 'click') : interval(0); // fallback

// Fecha janela ao clicar ou após 5 segundos, o que vier primeiro
const autoClose$ = timer(5000); // Emite após 5 segundos
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Janela sempre fechará dentro de 5 segundos
  mergeAll()
).subscribe();
```

#### ✅ Bom Exemplo 2: Fechar Janelas Periodicamente

Feche janelas periodicamente mesmo sem cliques.

```ts
import { interval, fromEvent, timer, merge } from 'rxjs';
import { window, mergeAll } from 'rxjs';

const source$ = interval(100);
const button = document.querySelector('#start-button');
const clicks$ = fromEvent(button, 'click');

// Fecha janela ao clicar ou a cada 3 segundos
const autoClose$ = timer(3000, 3000); // Após primeiros 3 segundos, depois a cada 3 segundos
const trigger$ = merge(clicks$, autoClose$);

source$.pipe(
  window(trigger$), // Janela fecha a cada 3 segundos mesmo sem cliques
  mergeAll()
).subscribe();

// Resultado:
// - Janelas fecham automaticamente a cada 3 segundos mesmo sem cliques do usuário
// - Previne acúmulo infinito de valores na memória
```

### 3. Sobreposição de Janela

Por padrão, janelas não se sobrepõem (próxima janela começa após fechamento da anterior).
Se sobreposição for necessária, use `windowToggle` ou `windowWhen`.

## 🆚 Comparação de Operadores window

| Operador | Momento da Delimitação | Caso de Uso |
|:---|:---|:---|
| `window` | Outro Observable emite | Particionamento orientado a eventos |
| `windowTime` | Intervalo de tempo fixo | Particionamento baseado em tempo |
| `windowCount` | Contagem fixa | Particionamento baseado em contagem |
| `windowToggle` | Observables de início e fim | Controle dinâmico de início/fim |
| `windowWhen` | Condição de fechamento dinâmica | Condição de fim diferente por janela |

## 📚 Operadores Relacionados

- [`buffer`](/pt/guide/operators/transformation/buffer) - Coleta valores como array (versão array de window)
- [`windowTime`](/pt/guide/operators/transformation/windowTime) - Particionamento de janela baseado em tempo
- [`windowCount`](/pt/guide/operators/transformation/windowCount) - Particionamento de janela baseado em contagem
- [`windowToggle`](/pt/guide/operators/transformation/windowToggle) - Controle de janela com Observables de início e fim
- [`windowWhen`](/pt/guide/operators/transformation/windowWhen) - Particionamento de janela com condição de fechamento dinâmica
- [`groupBy`](/pt/guide/operators/transformation/groupBy) - Agrupar Observables por chave

## Resumo

O operador `window` é uma ferramenta poderosa que divide streams acionados por um Observable externo e pode processar cada grupo como um Observable independente.

- ✅ Pode aplicar processamento diferente a cada janela
- ✅ Controle flexível orientado a eventos
- ✅ Suporta operações avançadas de stream
- ⚠️ Gerenciamento de inscrição necessário
- ⚠️ Cuidado com vazamentos de memória
