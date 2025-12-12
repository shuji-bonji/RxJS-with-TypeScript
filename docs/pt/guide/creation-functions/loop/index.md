---
description: Esta seção descreve Creation Functions que geram valores de forma semelhante a loops, usando range e generate para aprender como implementar processamento iterativo, como as instruções for e while, como streams Observable. Desde a geração sequencial de números até transições de estado complexas baseadas em condições personalizadas, você pode realizar processamento de loop declarativo utilizando a inferência de tipos do TypeScript.
---

# Loop Generation Creation Functions

Creation Functions para expressar processamento de loop, como as instruções for e while, como Observable.

## O que são Loop Generation Creation Functions?

Loop Generation Creation Functions realizam de forma reativa o processamento repetitivo. Ao substituir loops imperativos convencionais (instruções `for` e `while`) por streams Observable declarativos, é possível realizar processamento flexível em combinação com a cadeia de operadores do RxJS.

Confira a tabela abaixo para ver as características e o uso de cada Creation Function.

## Principais Loop Generation Creation Functions

| Function | Descrição | Casos de Uso |
|----------|------|-------------|
| **[range](/pt/guide/creation-functions/loop/range)** | Gerar um intervalo de números (como instrução for) | Geração sequencial de números, processamento em lote |
| **[generate](/pt/guide/creation-functions/loop/generate)** | Geração de loop de uso geral (como instrução while) | Repetição condicional, transições complexas de estado |

## Critérios de Uso

A seleção de Loop Generation Creation Functions é determinada a partir das seguintes perspectivas.

### 1. Padrão de Geração

- **Sequência numérica**: `range()` - Geração sequencial simples de números com valores de início e fim
- **Condições complexas**: `generate()` - Controle livre sobre valores iniciais, condições, iteração e seleção de resultados

### 2. Tipos de Loop

- **Loop semelhante à instrução for**: `range()` - `for (let i = start; i <= end; i++)`
- **Loop semelhante à instrução while**: `generate()` - `while (condition) { ... }`

### 3. Flexibilidade

- **Simples é suficiente**: `range()` - Quando você precisa de uma sequência de números
- **Necessita controle avançado**: `generate()` - Gerenciamento de estado personalizado, ramificação condicional, controle de passo

## Exemplos de Uso Prático

### range() - Geração Sequencial de Números

Para geração sequencial simples de números, `range()` é a melhor escolha.

```typescript
import { range, map } from 'rxjs';
// Gerar números sequenciais de 1 a 5
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Usar em processamento em lote
range(0, 10).pipe(
  map(i => `Processo ${i + 1}`)
).subscribe(console.log);
// Saída: Processo 1, Processo 2, ..., Processo 10
```

### generate() - Loop Condicional

Use `generate()` para condições complexas ou gerenciamento de estado personalizado.

```typescript
import { generate } from 'rxjs';

// Gerar sequência de Fibonacci (primeiros 10 termos)
generate(
  { current: 0, next: 1, count: 0 },  // Estado inicial
  state => state.count < 10,           // Condição de continuação
  state => ({                          // Atualização de estado
    current: state.next,
    next: state.current + state.next,
    count: state.count + 1
  }),
  state => state.current               // Seletor de resultado
).subscribe(console.log);
// Saída: 0, 1, 1, 2, 3, 5, 8, 13, 21, 34
```

## Comparação com Loop Imperativo

Esta é uma comparação entre o loop imperativo convencional e as Loop Generation Creation Functions do RxJS.

### Instrução for Imperativa

```typescript
// Instrução for convencional
const results: number[] = [];
for (let i = 1; i <= 5; i++) {
  results.push(i * 2);
}
console.log(results); // [2, 4, 6, 8, 10]
```

### range() Declarativo

```typescript
import { range, map, toArray } from 'rxjs';
// RxJS range()
range(1, 5).pipe(
  map(i => i * 2),
  toArray()
).subscribe(console.log); // [2, 4, 6, 8, 10]
```

> [!TIP]
> **Vantagens da abordagem declarativa**:
> - Legibilidade aprimorada com processamento em pipeline
> - Tratamento uniforme de erros
> - Fácil de combinar com processamento assíncrono
> - Fácil de cancelar e abortar (por exemplo, `takeUntil()`)

## Conversão de Cold para Hot

Como mostrado na tabela acima, **todas as Loop Generation Creation Functions geram Cold Observables**. Cada assinatura inicia uma execução independente.

No entanto, ao usar operadores de multicast (`share()`, `shareReplay()`, etc.), você pode **converter um Cold Observable em um Hot Observable**.

### Exemplo Prático: Compartilhamento de Resultados de Cálculo

```typescript
import { range, map, share } from 'rxjs';
// ❄️ Cold - Cálculo independente para cada assinatura
const cold$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calculando:', n);
    return n * n;
  })
);

cold$.subscribe(val => console.log('Assinante 1:', val));
cold$.subscribe(val => console.log('Assinante 2:', val));
// → Cálculo executado duas vezes (2000 cálculos)

// 🔥 Hot - Compartilhar resultados de cálculo entre assinantes
const hot$ = range(1, 1000).pipe(
  map(n => {
    console.log('Calculando:', n);
    return n * n;
  }),
  share()
);

hot$.subscribe(val => console.log('Assinante 1:', val));
hot$.subscribe(val => console.log('Assinante 2:', val));
// → Cálculo executado apenas uma vez (1000 cálculos)
```

> [!TIP]
> **Casos em que a conversão Hot é necessária**:
> - Usar cálculos de alto custo em vários locais
> - Compartilhar resultados de processamento em lote com vários componentes
> - Exibir resultados de paginação em vários componentes de UI
>
> Para mais informações, consulte [Basic Creation - Conversão de Cold para Hot](/pt/guide/creation-functions/basic/#converting-cold-to-hot).

## Combinado com Processamento Assíncrono

Loop Generation Creation Functions demonstram funcionalidade poderosa quando combinadas com processamento assíncrono.

### Execução Sequencial de Chamadas API

```typescript
import { range, of, Observable, concatMap, delay } from 'rxjs';
interface PageData {
  page: number;
  items: string[];
}

// Função para simular busca de dados de página
function fetchPage(page: number): Observable<PageData> {
  return of({
    page,
    items: [`Dados${page}-1`, `Dados${page}-2`, `Dados${page}-3`]
  }).pipe(
    delay(300) // Simular chamada API
  );
}

// Buscar sequencialmente as páginas 1 a 10 (com atraso de 1 segundo entre cada solicitação)
range(1, 10).pipe(
  concatMap(page =>
    fetchPage(page).pipe(delay(1000))
  )
).subscribe(
  data => console.log(`Página ${data.page} buscada:`, data.items),
  err => console.error('Erro:', err)
);
```

### Uso em Processamento de Retry

```typescript
import { range, throwError, of, Observable, mergeMap, retryWhen, delay } from 'rxjs';
// Função para simular busca de dados (falha aleatoriamente)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.6; // Taxa de sucesso de 40%

  return of(shouldFail).pipe(
    delay(200),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Falha na busca de dados'))
        : of('Busca de dados bem-sucedida')
    )
  );
}

function fetchWithRetry() {
  return fetchData().pipe(
    retryWhen(errors =>
      errors.pipe(
        mergeMap((error, index) => {
          // Tentar novamente até 3 vezes
          if (index >= 3) {
            return throwError(() => error);
          }
          console.log(`Tentativa ${index + 1}/3`);
          // Backoff exponencial: 1s, 2s, 4s
          return range(0, 1).pipe(delay(Math.pow(2, index) * 1000));
        })
      )
    )
  );
}

fetchWithRetry().subscribe({
  next: result => console.log('Resultado:', result),
  error: err => console.error('Erro:', err.message)
});

// Exemplo de saída:
// Tentativa 1/3
// Tentativa 2/3
// Resultado: Busca de dados bem-sucedida
```

## Relação com Pipeable Operator

Loop Generation Creation Functions não possuem um Pipeable Operator correspondente direto. Elas são sempre usadas como Creation Functions.

No entanto, processamento mais avançado é possível combinando-as com os seguintes operadores:

| Operadores para Combinar | Propósito |
|-------------------|------|
| `map()` | Transformar cada valor |
| `filter()` | Passar apenas valores que correspondem à condição |
| `take()`, `skip()` | Controlar o número de valores |
| `concatMap()`, `mergeMap()` | Executar processamento assíncrono para cada valor |
| `toArray()` | Coletar todos os valores em um array |

## Notas de Desempenho

Loop Generation Creation Functions emitem valores de forma síncrona, portanto, tenha cuidado com o desempenho ao gerar um grande número de valores.

> [!WARNING]
> **Tratamento de grandes quantidades de dados**:
> - Grandes quantidades de dados, como `range(1, 1000000)`, são emitidas de forma síncrona e consomem memória
> - Use buffer com `bufferCount()` ou `windowCount()` conforme necessário
> - Ou mude para execução assíncrona especificando um scheduler com `scheduled()`

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
// Executar com scheduler assíncrono
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe(console.log);
```

## Próximos Passos

Para aprender mais sobre o comportamento detalhado e exemplos práticos de cada Creation Function, clique nos links da tabela acima.

Você também pode entender o panorama geral das Creation Functions aprendendo [Basic Creation Functions](/pt/guide/creation-functions/basic/), [Combination Creation Functions](/pt/guide/creation-functions/combination/), [Selection/Partition Creation Functions](/pt/guide/creation-functions/selection/) e [Conditional Creation Functions](/pt/guide/creation-functions/conditional/).
