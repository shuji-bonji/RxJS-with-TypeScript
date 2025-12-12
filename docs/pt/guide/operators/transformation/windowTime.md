---
description: windowTime é um operador RxJS que pode dividir um Observable em intervalos de tempo regulares e processar valores emitidos em cada período de tempo como um Observable separado.
titleTemplate: ':title | RxJS'
---

# windowTime - Divide Observable em Intervalos de Tempo Regulares

O operador `windowTime` agrupa os valores do Observable fonte **em intervalos regulares** e gera esse grupo como um **novo Observable**.
Enquanto `bufferTime` retorna um array, `windowTime` retorna um **Observable&lt;T&gt;**, permitindo que operadores adicionais sejam aplicados a cada janela.

## 🔰 Sintaxe Básica e Uso

```ts
import { interval } from 'rxjs';
import { windowTime, mergeAll, take } from 'rxjs';

// Emite valores a cada 100ms
const source$ = interval(100);

source$.pipe(
  windowTime(1000), // Cria janela a cada 1 segundo
  take(3),          // Apenas primeiras 3 janelas
  mergeAll()        // Achata cada janela
).subscribe(value => {
  console.log('Valor:', value);
});

// Saída:
// 1º segundo: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
// 2º segundo: 10, 11, 12, 13, 14, 15, 16, 17, 18, 19
// 3º segundo: 20, 21, 22, 23, 24, 25, 26, 27, 28, 29
```

- Uma nova janela (Observable) é criada a cada tempo especificado (1000ms).
- Cada janela pode ser processada como um Observable independente.

[🌐 Documentação Oficial RxJS - `windowTime`](https://rxjs.dev/api/operators/windowTime)

## 💡 Padrões Típicos de Uso

- **Processamento em lote baseado em tempo**: Dados são processados em lotes em intervalos regulares
- **Agregar dados em tempo real**: Contar o número de eventos por segundo
- **Monitoramento de desempenho**: Coletar métricas em intervalos regulares
- **Análise de dados de série temporal**: Processamento estatístico por período de tempo

## 🔍 Diferença de bufferTime

| Operador | Saída | Caso de Uso |
|:---|:---|:---|
| `bufferTime` | **Array (T[])** | Processar valores agrupados juntos |
| `windowTime` | **Observable&lt;T&gt;** | Processamento de stream diferente para cada período de tempo |

```ts
import { interval } from 'rxjs';
import { bufferTime, windowTime, take } from 'rxjs';

const source$ = interval(100);

// bufferTime - Gera como array
source$.pipe(
  bufferTime(1000),
  take(2)
).subscribe(values => {
  console.log('Buffer (array):', values);
  // Saída: Buffer (array): [0, 1, 2, 3, 4, 5, 6, 7, 8, 9]
});

// windowTime - Gera como Observable
source$.pipe(
  windowTime(1000),
  take(2)
).subscribe(window$ => {
  console.log('Window (Observable):', window$);
  window$.subscribe(value => {
    console.log('  Valor:', value);
  });
});
```

## 🧠 Exemplo de Código Prático 1: Contar Cliques Por Segundo

Este é um exemplo de contar o número de cliques em um botão a cada segundo.

```ts
import { fromEvent } from 'rxjs';
import { windowTime, map, mergeAll, scan } from 'rxjs';

// Criar botão
const button = document.createElement('button');
button.textContent = 'Clicar';
document.body.appendChild(button);

// Área de saída
const output = document.createElement('div');
output.style.marginTop = '10px';
document.body.appendChild(output);

// Evento de clique
const clicks$ = fromEvent(button, 'click');

let windowNumber = 0;

clicks$.pipe(
  windowTime(1000), // Cria janela a cada 1 segundo
  map(window$ => {
    ++windowNumber;

    // Conta cliques em cada janela
    return window$.pipe(
      scan(count => count + 1, 0)
    );
  }),
  mergeAll()
).subscribe(count => {
  const timestamp = new Date().toLocaleTimeString();
  output.textContent = `[${timestamp}] Janela ${windowNumber}: ${count} cliques`;
});
```

- Uma nova janela é criada a cada segundo.
- O número de cliques em cada janela é contado em tempo real.

## 🎯 Exemplo de Código Prático 2: Processamento Estatístico por Período de Tempo

Este exemplo calcula a soma e média dos valores para cada período de tempo.

```ts
import { interval } from 'rxjs';
import { windowTime, map, mergeMap, toArray, take } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Processamento Estatístico por Período de Tempo (a cada 1 segundo)</h3>';
document.body.appendChild(output);

const table = document.createElement('table');
table.style.borderCollapse = 'collapse';
table.style.marginTop = '10px';
table.innerHTML = `
  <thead>
    <tr style="background: #f0f0f0;">
      <th style="border: 1px solid #ccc; padding: 8px;">Janela</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Contagem</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Soma</th>
      <th style="border: 1px solid #ccc; padding: 8px;">Média</th>
    </tr>
  </thead>
  <tbody id="stats-body"></tbody>
`;
output.appendChild(table);

const source$ = interval(100).pipe(
  map(() => Math.floor(Math.random() * 100)) // Valor aleatório
);

let windowNumber = 0;

source$.pipe(
  windowTime(1000), // A cada 1 segundo
  take(5),          // Apenas 5 janelas
  mergeMap(window$ => {
    const current = ++windowNumber;

    // Converte valores em cada janela para array e processa estatísticas
    return window$.pipe(
      toArray(),
      map(values => ({
        window: current,
        count: values.length,
        sum: values.reduce((a, b) => a + b, 0),
        avg: values.length > 0
          ? (values.reduce((a, b) => a + b, 0) / values.length).toFixed(2)
          : 0
      }))
    );
  })
).subscribe(stats => {
  const tbody = document.getElementById('stats-body')!;
  const row = document.createElement('tr');
  row.innerHTML = `
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.window}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.count}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.sum}</td>
    <td style="border: 1px solid #ccc; padding: 8px; text-align: center;">${stats.avg}</td>
  `;
  tbody.appendChild(row);
});
```

- Estatísticas para cada janela podem ser calculadas separadamente.
- Processamento diferente pode ser aplicado a cada janela.
- Estatísticas são exibidas visualmente em formato de tabela.

## 📊 Janelas Sobrepostas (windowCreationInterval)

Você pode sobrepor janelas especificando `windowCreationInterval` como o segundo argumento.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray, take, map } from 'rxjs';

// Criar área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Janelas Sobrepostas</h3>';
output.style.border = '1px solid #ccc';
output.style.padding = '10px';
output.style.marginTop = '10px';
document.body.appendChild(output);

const source$ = interval(100);

source$.pipe(
  windowTime(
    2000,  // Comprimento da janela: 2 segundos
    1000   // Intervalo de criação de janela: 1 segundo
  ),
  take(3),
  mergeMap((window$, index) =>
    window$.pipe(
      toArray(),
      map(values => ({ window: index + 1, values }))
    )
  )
).subscribe(result => {
  const div = document.createElement('div');
  div.style.marginTop = '10px';
  div.style.padding = '5px';
  div.style.backgroundColor = '#f5f5f5';
  div.style.borderLeft = '3px solid #4CAF50';

  const title = document.createElement('strong');
  title.textContent = `Janela ${result.window}:`;
  div.appendChild(title);

  div.appendChild(document.createElement('br'));

  const values = document.createElement('span');
  values.textContent = `Valores: [${result.values.join(', ')}]`;
  div.appendChild(values);

  div.appendChild(document.createElement('br'));

  const info = document.createElement('span');
  info.style.color = '#666';
  info.textContent = `(${result.values.length} valores, ${(result.window - 1)} seg ~ ${(result.window + 1)} seg)`;
  div.appendChild(info);

  output.appendChild(div);

  // Chrome workaround: Força renderização
  void output.offsetHeight;
});
```

**Como funciona:**
- **Janela 1**: Valores de 0 a 2 segundos `[0, 1, 2, ..., 19]` (20 valores)
- **Janela 2**: Valores de 1 a 3 segundos `[10, 11, 12, ..., 29]` (20 valores) ← Valores 10-19 se sobrepõem com Janela 1
- **Janela 3**: Valores de 2 a 4 segundos `[20, 21, 22, ..., 39]` (20 valores) ← Valores 20-29 se sobrepõem com Janela 2

- Criar uma nova janela com um intervalo (1 segundo) menor que o comprimento da janela (2 segundos) resultará em sobreposição.
- Útil para implementações de janela deslizante.

## 🎯 Exemplo Prático: Monitoramento de Eventos em Tempo Real

```ts
import { fromEvent } from 'rxjs';
import { windowTime, mergeMap, toArray, map } from 'rxjs';

// Área de saída
const output = document.createElement('div');
output.innerHTML = '<h3>Monitoramento de Movimento do Mouse (a cada 5 segundos)</h3>';
document.body.appendChild(output);

const list = document.createElement('ul');
output.appendChild(list);

// Evento de movimento do mouse
const moves$ = fromEvent<MouseEvent>(document, 'mousemove');

moves$.pipe(
  windowTime(5000), // A cada 5 segundos
  mergeMap(window$ =>
    window$.pipe(
      toArray(),
      map(events => ({
        count: events.length,
        timestamp: new Date().toLocaleTimeString()
      }))
    )
  )
).subscribe(result => {
  const item = document.createElement('li');
  item.textContent = `[${result.timestamp}] Movimentos do mouse: ${result.count} vezes`;
  list.insertBefore(item, list.firstChild);

  // Exibe até 10 itens
  while (list.children.length > 10) {
    list.removeChild(list.lastChild!);
  }
});
```

## ⚠️ Notas

### 1. Gerenciamento de Inscrição de Janela

Cada janela é um Observable independente e deve ser explicitamente inscrita.

```ts
source$.pipe(
  windowTime(1000)
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
  windowTime(1000),
  mergeAll() // Mescla todas as janelas
).subscribe(value => {
  console.log('Valor:', value);
});
```

### 2. Gerenciamento de Memória

Ao executar por longos períodos de tempo, é importante cancelar a inscrição adequadamente.

```ts
import { takeUntil } from 'rxjs';
import { Subject } from 'rxjs';

const destroy$ = new Subject<void>();

source$.pipe(
  windowTime(1000),
  mergeAll(),
  takeUntil(destroy$) // Cancela inscrição ao destruir
).subscribe();

// Quando componente é destruído, etc.
destroy$.next();
destroy$.complete();
```

### 3. Especificar Valor Máximo (maxWindowSize)

O terceiro argumento permite limitar o número máximo de valores em cada janela.

```ts
import { interval } from 'rxjs';
import { windowTime, mergeMap, toArray } from 'rxjs';

interval(100).pipe(
  windowTime(
    2000,      // Comprimento da janela: 2 segundos
    undefined, // Intervalo de criação de janela: padrão (sem sobreposição)
    5          // Contagem máxima de valores: até 5
  ),
  mergeMap(window$ => window$.pipe(toArray()))
).subscribe(values => {
  console.log('Janela:', values);
  // Contém no máximo apenas 5 valores
});
```

## 🆚 Comparação de Operadores window

| Operador | Momento da Delimitação | Caso de Uso |
|:---|:---|:---|
| `window` | Outro Observable emite | Particionamento orientado a eventos |
| `windowTime` | **Intervalo de tempo fixo** | **Particionamento baseado em tempo** |
| `windowCount` | Contagem fixa | Particionamento baseado em contagem |
| `windowToggle` | Observables de início e fim | Controle dinâmico de início/fim |
| `windowWhen` | Condição de fechamento dinâmica | Condição de fim diferente por janela |

## 📚 Operadores Relacionados

- [bufferTime](/pt/guide/operators/transformation/bufferTime) - Coleta valores como array (versão array de windowTime)
- [window](/pt/guide/operators/transformation/window) - Divide janela em momentos diferentes de Observable
- [windowCount](/pt/guide/operators/transformation/windowCount) - Divisão de janela baseada em contagem
- [windowToggle](/pt/guide/operators/transformation/windowToggle) - Controle de janela com Observables de início e fim
- [windowWhen](/pt/guide/operators/transformation/windowWhen) - Divisão de janela com condições de fechamento dinâmicas

## Resumo

O operador `windowTime` é uma ferramenta poderosa para dividir streams em base de tempo e tratar cada período de tempo como um Observable independente.

- ✅ Cria janelas automaticamente em intervalos regulares
- ✅ Processamento diferente pode ser aplicado a cada janela
- ✅ Suporta janelas deslizantes (sobreposição)
- ✅ Ideal para agregação e análise de dados em tempo real
- ⚠️ Gerenciamento de inscrição necessário
- ⚠️ Esteja ciente do gerenciamento de memória
