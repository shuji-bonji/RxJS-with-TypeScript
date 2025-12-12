---
description: "range() - Creation Function que gera inteiros consecutivos de forma declarativa: Alternativa eficiente em memória aos loops for para processamento em lote e paginação"
---

# range() - Gera um intervalo de números

`range()` é uma Creation Function semelhante à instrução for que emite um número especificado de inteiros consecutivos a partir de um valor inicial especificado.

## Visão Geral

`range()` emite uma sequência de inteiros consecutivos como Observable especificando um valor inicial e o número de inteiros. É usado para geração sequencial de números e processamento em lote como uma maneira declarativa de substituir a instrução `for` tradicional.

**Assinatura**:
```typescript
function range(
  start: number,
  count?: number,
  scheduler?: SchedulerLike
): Observable<number>
```

**Parâmetros**:
- `start`: O valor inicial (a partir do qual começar a emitir)
- `count`: o número de valores a publicar (omitido, de 0 a menos de `start`)
- `scheduler`: o scheduler para emitir os valores (omitido: emitir de forma síncrona)

**Documentação Oficial**: [📘 RxJS Oficial: range()](https://rxjs.dev/api/index/function/range)

## Uso Básico

### Padrão 1: Especificar valor inicial e contagem

Este é o uso mais comum.

```typescript
import { range } from 'rxjs';

// Gerar 5 números sequenciais a partir de 1 (1, 2, 3, 4, 5)
range(1, 5).subscribe({
  next: value => console.log('Valor:', value),
  complete: () => console.log('Completo')
});

// Saída:
// Valor: 1
// Valor: 2
// Valor: 3
// Valor: 4
// Valor: 5
// Completo
```

### Padrão 2: Números sequenciais começando de 0

Ao definir o valor inicial como 0, um número sequencial como um índice de array pode ser gerado.

```typescript
import { range } from 'rxjs';

// 0 a 10 números sequenciais (0, 1, 2, ..., 9)
range(0, 10).subscribe(console.log);
// Saída: 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
```

### Padrão 3: Começar com um número negativo

Números negativos também podem ser gerados.

```typescript
import { range } from 'rxjs';

// 5 números sequenciais a partir de -3 (-3, -2, -1, 0, 1)
range(-3, 5).subscribe(console.log);
// Saída: -3, -2, -1, 0, 1
```

## Características Importantes

### 1. Emissão Síncrona

Por padrão, `range()` emite todos os valores **de forma síncrona** após a assinatura.

```typescript
import { range } from 'rxjs';

console.log('Antes da assinatura');

range(1, 3).subscribe(value => console.log('Valor:', value));

console.log('Após a assinatura');

// Saída:
// Antes da assinatura
// Valor: 1
// Valor: 2
// Valor: 3
// Após a assinatura
```

### 2. Completa Imediatamente

Notifica `complete` imediatamente após publicar todos os valores.

```typescript
import { range } from 'rxjs';

range(1, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Completo!')
});

// Saída: 1, 2, 3, Completo!
```

### 3. Equivalência com a instrução for

`range(start, count)` é equivalente à seguinte instrução for.

```typescript
// Instrução for imperativa
for (let i = start; i < start + count; i++) {
  console.log(i);
}

// range() declarativo
range(start, count).subscribe(console.log);
```

## Casos de Uso Práticos

### 1. Processamento em Lote

Usado para executar várias tarefas sequencialmente.

```typescript
import { range, of, Observable, concatMap, delay, map } from 'rxjs';
// Função para simular processamento de dados
function processItem(index: number): Observable<string> {
  return of(index).pipe(
    delay(100), // Simular tempo de processamento de 100ms
    map(i => `Resultado do processamento do item ${i}`)
  );
}

// Processar sequencialmente 10 itens de dados (atraso de 1 segundo entre cada processo)
range(1, 10).pipe(
  concatMap(index =>
    processItem(index).pipe(delay(1000))
  )
).subscribe({
  next: result => console.log(`Processamento completo: ${result}`),
  complete: () => console.log('Todo o processamento concluído')
});

// Saída:
// Processamento completo: Resultado do processamento do item 1 (após cerca de 1,1 segundos)
// Processamento completo: Resultado do processamento do item 2 (após cerca de 2,1 segundos)
// ...
// Processamento completo: Resultado do processamento do item 10 (após aprox. 10,1 seg.)
// Todo o processamento está completo
```

### 2. Paginação

Recuperar múltiplas páginas de dados sequencialmente.

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
    items: [`Item${page}-1`, `Item${page}-2`, `Item${page}-3`]
  }).pipe(
    delay(500) // Simular chamada API
  );
}

function fetchAllPages(totalPages: number) {
  return range(1, totalPages).pipe(
    concatMap(page => fetchPage(page))
  );
}

fetchAllPages(5).subscribe({
  next: (data: PageData) => console.log(`Página ${data.page}:`, data.items),
  complete: () => console.log('Todas as páginas recuperadas')
});

// Saída:
// Página 1: ['Item1-1', 'Item1-2', 'Item1-3']
// Página 2: ['Item2-1', 'Item2-2', 'Item2-3']
// Página 3: ['Item3-1', 'Item3-2', 'Item3-3']
// Página 4: ['Item4-1', 'Item4-2', 'Item4-3']
// Página 5: ['Item5-1', 'Item5-2', 'Item5-3']
// Todas as páginas recuperadas
```

### 3. Processamento de Índices de Array

Use como um loop baseado em índice ao processar cada elemento de um array.

```typescript
import { range, map } from 'rxjs';
const items = ['Maçã', 'Banana', 'Cereja', 'Tâmara', 'Sabugueiro'];

range(0, items.length).pipe(
  map(index => ({ index, item: items[index] }))
).subscribe(({ index, item }) => {
  console.log(`[${index}] ${item}`);
});

// Saída:
// [0] Maçã
// [1] Banana
// [2] Cereja
// [3] Tâmara
// [4] Sabugueiro
```

### 4. Geração de Dados de Teste

Isso é útil para gerar dados mock para testes unitários.

```typescript
import { range, map, toArray } from 'rxjs';
// Gerar dados de usuário mock
range(1, 100).pipe(
  map(id => ({
    id,
    name: `Usuario${id}`,
    email: `usuario${id}@exemplo.com`
  })),
  toArray()
).subscribe(users => {
  console.log(`${users.length} usuários gerados`);
  // Usar em testes
});
```

### 5. Contador para Processamento de Retry

Controla o número de tentativas em caso de erro.

```typescript
import { range, throwError, concat, of, Observable, mergeMap, delay, catchError, map, toArray } from 'rxjs';
// Função para simular busca de dados (falha aleatoriamente)
function fetchData(): Observable<string> {
  const shouldFail = Math.random() > 0.7; // 30% de chance de sucesso

  return of(shouldFail).pipe(
    delay(300),
    mergeMap(fail =>
      fail
        ? throwError(() => new Error('Falha na aquisição de dados'))
        : of('Aquisição de dados bem-sucedida')
    )
  );
}

function fetchWithRetry(maxRetries: number = 3) {
  return concat(
    range(0, maxRetries).pipe(
      map(attempt => {
        console.log(`Tentativa ${attempt + 1}/${maxRetries}`);
        return fetchData().pipe(
          catchError(error => {
            if (attempt === maxRetries - 1) {
              return throwError(() => new Error('Número máximo de tentativas atingido'));
            }
            return throwError(() => error);
          }),
          delay(Math.pow(2, attempt) * 1000) // Backoff exponencial
        );
      }),
      toArray()
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
// Resultado: Aquisição de dados bem-sucedida
```

## Assincronização com Scheduler

Ao processar grandes quantidades de dados, a execução assíncrona é possível especificando um scheduler.

```typescript
import { range, asyncScheduler, observeOn } from 'rxjs';
console.log('Início');

// Emitir de forma assíncrona 1.000.000 números
range(1, 1000000).pipe(
  observeOn(asyncScheduler)
).subscribe({
  next: val => {
    if (val % 100000 === 0) {
      console.log(`Progresso: ${val}`);
    }
  },
  complete: () => console.log('Completo')
});

console.log('Após a assinatura (assíncrono, então é executado imediatamente)');

// Saída:
// Início
// Após a assinatura (assíncrono, então será executado imediatamente)
// Progresso: 100000
// Progresso: 200000
// ...
// Completo
```

> [!TIP]
> **Usando o Scheduler**:
> - Não bloquear a UI ao processar grandes quantidades de dados
> - Controle de tempo em testes (TestScheduler)
> - Controle de event loop em ambiente Node.js

Para mais informações, consulte [Tipos de Schedulers e Como Usá-los](/pt/guide/schedulers/types).

## Comparação com Outras Creation Functions

### range() vs of()

```typescript
import { range, of } from 'rxjs';

// range() - inteiros consecutivos
range(1, 3).subscribe(console.log);
// Saída: 1, 2, 3

// of() - enumerar valores arbitrários
of(1, 2, 3).subscribe(console.log);
// Saída: 1, 2, 3

// Diferença: range() aceita apenas números sequenciais, of() aceita valores arbitrários
of(1, 10, 100).subscribe(console.log);
// Saída: 1, 10, 100 (não é possível com range())
```

### range() vs from()

```typescript
import { range, from } from 'rxjs';

// range() - gerar números sequenciais
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// from() - gerar a partir de um array (deve criar array antecipadamente)
from([1, 2, 3, 4, 5]).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Vantagem do range(): sem pré-alocação de arrays na memória
range(1, 1000000); // Eficiente em memória
from(Array.from({ length: 1000000 }, (_, i) => i + 1)); // Array vai para a memória
```

### range() vs generate()

```typescript
import { range, generate } from 'rxjs';

// range() - numeração sequencial simples
range(1, 5).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// generate() - um exemplo complexo da mesma coisa
generate(
  1,                    // Valor inicial
  x => x <= 5,          // Condição de continuação
  x => x + 1            // Iteração
).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5

// Vantagens do generate(): gerenciamento complexo de condições e estado
generate(
  1,
  x => x <= 100,
  x => x * 2  // Incrementa por um fator de 2
).subscribe(console.log);
// Saída: 1, 2, 4, 8, 16, 32, 64
// (não é possível com range())
```

> [!TIP]
> **Critérios de Seleção**:
> - **Requer números sequenciais** → `range()`
> - **Enumerar qualquer valor** → `of()`
> - **Array/Promise existente** → `from()`
> - **Condição/passo complexo** → `generate()`

## Considerações de Desempenho

Como `range()` emite valores de forma síncrona, o desempenho deve ser considerado ao gerar grandes números de valores.

> [!WARNING]
> **Tratamento de Grandes Quantidades de Dados**:
> ```typescript
> // ❌ Exemplo ruim: emitir 1 milhão de valores de forma síncrona (UI será bloqueada)
> range(1, 1000000).subscribe(console.log);
>
> // ✅ Bom exemplo 1: assíncrono com scheduler
> range(1, 1000000).pipe(
>   observeOn(asyncScheduler)
> ).subscribe(console.log);
>
> // ✅ Bom Exemplo 2: Dividir por buffer
> range(1, 1000000).pipe(
>   bufferCount(1000)
> ).subscribe(batch => console.log(`${batch.length} casos processados`));
> ```

## Escolha Entre from() Array

```typescript
import { range, from } from 'rxjs';

// Se você precisa de números sequenciais → range() é mais conciso
range(0, 10).subscribe(console.log);

// Não há necessidade de criar um array e depois convertê-lo (ineficiente)
from(Array.from({ length: 10 }, (_, i) => i)).subscribe(console.log);

// Se houver um array existente → use from()
const existingArray = [5, 10, 15, 20];
from(existingArray).subscribe(console.log);
```

## Tratamento de Erros

Embora `range()` em si não emita erros, erros podem ocorrer no pipeline.

```typescript
import { range, of, map, catchError } from 'rxjs';
range(1, 10).pipe(
  map(n => {
    if (n === 5) {
      throw new Error('Erro em 5');
    }
    return n * 2;
  }),
  catchError(error => {
    console.error('Erro ocorreu:', error.message);
    return of(-1); // Retornar valor padrão
  })
).subscribe(console.log);

// Saída: 2, 4, 6, 8, -1
```

## Resumo

`range()` é uma Creation Function simples, mas poderosa, que produz uma sequência de inteiros consecutivos.

> [!IMPORTANT]
> **Recursos do range()**:
> - ✅ Ideal para gerar números consecutivos (alternativa à instrução for)
> - ✅ Útil para processamento em lote, paginação, geração de dados de teste
> - ✅ Eficiente em memória (sem pré-criação de arrays)
> - ⚠️ Considere assíncrono para grandes quantidades de dados
> - ⚠️ Use `generate()` para condições complexas

## Tópicos Relacionados

- [generate()](/pt/guide/creation-functions/loop/generate) - Geração de loop genérico
- [of()](/pt/guide/creation-functions/basic/of) - Enumera valores arbitrários
- [from()](/pt/guide/creation-functions/basic/from) - Converter de array ou Promise
- [interval()](/pt/guide/creation-functions/basic/interval) - Publicar valores periodicamente

## Referências

- [RxJS Oficial: range()](https://rxjs.dev/api/index/function/range)
- [Learn RxJS: range](https://www.learnrxjs.io/learn-rxjs/operators/creation/range)
