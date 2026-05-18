---
description: "Creation Function que geram valores em uma maneira semelhante a um loop serão explicadas: saiba como usar range e generate para implementar processos iterativos, como instruções for e while, como fluxos Observable. Da geração de números sequenciais a transições de estado complexas com base em condições personalizadas, você pode realizar o processamento de loop declarativo que faz uso da inferência de tipo do TypeScript."
---

# Funções de criação do sistema de geração de loop

Creation Function para representar o processamento de loop, como instruções for e while, como Observable.

## Sistema de geração de loop O que são Creation Function?

As Creation Functions para sistemas de geração de loops realizam processos repetitivos de forma reativa. Ao substituir os loops imperativos tradicionais (instruções `for` e `while`) por fluxos Observable declarativos, elas permitem um processamento flexível em combinação com a cadeia de operadores do RxJS.

Consulte a tabela abaixo para ver as características e o uso de cada Creation Function.

## Sistemas de geração de loop principal Creation Function

| Funções | Descrição | Caso de uso. |
|---|---|---|
| **[range](/pt/guide/creation-functions/loop/range)** | Gerar um intervalo de números (para declarações) | Geração de números sequenciais, processamento em lote. |
| **[generate](/pt/guide/creation-functions/loop/generate)** | Gerar loops genéricos (tipo while statement) | Repetição condicional, transições de estado complexas |

## Critérios de uso

A escolha das Loop Generating Creation Function é determinada pelos seguintes aspectos.

### 1. Padrão de geração

- Sequência numérica**: `range()` - geração de números sequenciais simples com valores iniciais e finais.
- Condições complexas**: `generate()` - controle livre sobre valores iniciais, condições, iterações e seleção de resultados

### Tipos de loop

- Loop do tipo for statement**: `range()` - `for (let i = start; i <= end; i++)`
- **loops do tipo while**: `generate()` - `while (condition) { ... }`

### 3. flexibilidade

- Suficientemente simples**: `range()` - se for necessária uma sequência de números
- **Precisa de controle avançado**: `generate()` - gerenciamento de estado personalizado, ramificação condicional, controle de etapas

## Casos de uso práticos

### range() - geração de números sequenciais

Para a geração de números sequenciais simples, range()` é a melhor opção.

```typescript
import { range, map } from 'rxjs';
// 1para5Gerar números sequenciais de
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Uso no processamento em lote
range(0, 10).pipe(
  map(i => `Processamento${i + 1}`)
).subscribe(console.log);
// Saída: Processamento1, Processamento2, ..., Processamento10
```

### generate() - loop condicional

Use `generate()` quando forem necessárias condições complexas ou gerenciamento de estado personalizado.

```typescript
import { generate } from 'rxjs';

// Gerar a sequência de Fibonacci (primeiro10termo)
generate(
  { current: 0, next: 1, count: 0 },  // Condição inicial
  state => state.count < 10,           // Condição de continuação
  state => ({                          // Atualização do estado
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Seleção de resultados
).subscribe(console.log);
// Saída: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Comparado com o loop imperativo

Esta é uma comparação entre os loops imperativos convencionais e o sistema de geração de loops RxJS Creation Function.

### Declaração imperativa for

```typescript
// ConvencionalforSentença
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### Declarativo range()

```typescript
import { range, map, toArray } from 'rxjs';
// RxJSderange()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

```typescript
import { range, map } from 'rxjs';
// 1para5Gerar números sequenciais de
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Uso no processamento em lote
range(0, 10).pipe(
  map(i => `Processamento${i + 1}`)
).subscribe(console.log);
// Saída: Processamento1, Processamento2, ..., Processamento10
```

> **Benefícios da abordagem declarativa**:.
> - Melhor legibilidade devido ao processamento de pipeline
> - Tratamento uniforme de erros
> - Fácil de combinar com o processamento assíncrono
> - Fácil de cancelar e abortar (por exemplo, takeUntil()`)

## Conversão de frio para quente

Conforme mostrado na tabela acima, **todas as Creation Function geradoras de loop geram um Observable Cold**. Cada assinatura inicia uma execução independente.

No entanto, os **Cold Observable podem ser convertidos** em Hot Observable usando operadores multicast (`share()`, `shareReplay()`, etc.).

### Exemplo prático: compartilhar os resultados de um cálculo.


```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Cálculo independente por assinatura
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Cálculo em andamento:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Assinante1:', val));
cold$.subscribe(val => console.log('Assinante2:', val));
// → Um cálculo é2realizado uma vez (em2000Cálculo realizado uma vez)

// 🔥 Hot - Os resultados do cálculo são compartilhados entre os assinantes
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Cálculo em andamento:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Assinante1:', val));
hot$.subscribe(val => console.log('Assinante2:', val));
// → O cálculo é1Executado apenas uma vez (1000Cálculo realizado uma vez)
```

```typescript
import { range, map } from 'rxjs';
// 1para5Gerar números sequenciais de
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Uso no processamento em lote
range(0, 10).pipe(
  map(i => `Processamento${i + 1}`)
).subscribe(console.log);
// Saída: Processamento1, Processamento2, ..., Processamento10
```

> **Casos em que o aquecimento é necessário**:.
> - Cálculos de alto custo são usados em vários locais
> Resultados do processamento em lote compartilhados por vários componentes
> - Resultados do processo de paginação exibidos em vários componentes da interface do usuário.
>
> Para obter mais informações, consulte [Sistemas básicos de criação - Conversão de frio para quente](/pt/guide/creation-functions/basic/#cold- to -hot-).

## Combinado com o processamento assíncrono

O sistema de geração de loop Creation Function pode ser combinado com o processamento assíncrono para fornecer uma funcionalidade poderosa.

### Execução sequencial de chamadas de API


```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Função para simular a aquisição de dados de página
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Dados${page}-1`, `Dados${page}-2`, `Dados${page}-3`]
  }).pipe(
    delay(300) // APISimular uma chamada para
  );
}

// Página1para10Recupera sequencialmente até (com um atraso de1segundos de atraso entre cada solicitação)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe({
  next: data => console.log(`Página ${data.page} Obtenção:`, data.items),
  error: err => console.error('Erro:', err)
});
```

### Uso no processamento de repetição

```typescript
import { range, throwError, of, Observable, mergeMap, retry, delay } from 'rxjs';
// Função para simular a aquisição de dados (falha aleatória)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // 40%Sucesso com uma probabilidade de

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Falha na aquisição de dados'))
        : of('Aquisição de dados bem-sucedida')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    // RxJS 7.3+ Recomendação: retry({ count, delay }) Formato
    retry({
      count: 3, // Máx.3Repetições
      delay: (error, retryCount) => {
        console.log(`Novas tentativas ${retryCount}/3`);
        // Recuo exponencial: 1Segundos,2Segundos,4seg.
        return range(0, 1).pipe(delay(Math.pow(2, retryCount - 1) * 1000));
      }
    })
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultado:', result),
  error: err => console.error('Erro:', err.message)
});

// Exemplo de saída:
// Novas tentativas 1/3
// Novas tentativas 2/3
// Resultado: Aquisição de dados bem-sucedida
```

## Relação com o Pipeable Operator

As Creation Function geradoras de loop não têm uma contrapartida direta do Pipeable Operator. Elas são sempre usadas como Creation Function.

No entanto, elas podem ser combinadas com os seguintes operadores para um processamento mais avançado.

```typescript
import { range, map } from 'rxjs';
// 1para5Gerar números sequenciais de
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Uso no processamento em lote
range(0, 10).pipe(
  map(i => `Processamento${i + 1}`)
).subscribe(console.log);
// Saída: Processamento1, Processamento2, ..., Processamento10
```

## Notas de desempenho.

As Creation Function que geram loops emitem valores de forma síncrona, portanto, devem ser tomadas notas de desempenho ao gerar um grande número de valores.

> [!WARNING]

> **Tratamento de grandes quantidades de dados**:.
> - Grandes quantidades de dados, como range(1, 1000000)`, são emitidas de forma síncrona e consomem memória
> - Coloque o buffer com `bufferCount()` ou `windowCount()` conforme necessário
> ou altere para execução assíncrona especificando um agendador com `scheduled()`.


```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Executado pelo agendador assíncrono
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Próximas etapas.

Para saber mais sobre como cada Creation Function funciona e exemplos práticos, clique nos links da tabela acima.

Você também pode saber mais sobre [funções básicas de criação](/pt/guide/creation-functions/basic/), [funções de criação combinadas](/pt/guide/creation-functions/combination/), [seleção e Creation Function](/pt/guide/creation-functions/selection/) e [Creation Function](/pt/guide/creation-functions/conditional/). Isso o ajudará a entender o quadro completo das Creation Function.
