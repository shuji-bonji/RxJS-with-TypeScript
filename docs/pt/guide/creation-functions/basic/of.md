---
description: "of() - A Creation Function mais simples que emite valores especificados em sequência: Perfeita para dados de teste, mocks, valores padrão e ramificação condicional"
---

# of() - Emissão Sequencial de Valores

`of()` é a Creation Function mais simples que emite os valores especificados um a um em sequência.

## Visão Geral

`of()` emite os valores passados como argumentos em sequência conforme são subscritos, e completa imediatamente após todos os valores serem emitidos. É frequentemente usada para criar código de teste ou dados mock.

**Assinatura**:
```typescript
function of<T>(...args: T[]): Observable<T>
```

**Documentação Oficial**: [📘 RxJS Official: of()](https://rxjs.dev/api/index/function/of)

## Uso Básico

`of()` permite que múltiplos valores sejam passados separados por vírgula.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3, 4, 5);

values$.subscribe({
  next: value => console.log('Valor:', value),
  error: err => console.error('Erro:', err),
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

## Características Importantes

### 1. Emissão Síncrona

`of()` emite todos os valores **sincronamente** na assinatura.

```typescript
import { of } from 'rxjs';

console.log('Antes da assinatura');

of('A', 'B', 'C').subscribe(value => console.log('Valor:', value));

console.log('Depois da assinatura');

// Saída:
// Antes da assinatura
// Valor: A
// Valor: B
// Valor: C
// Depois da assinatura
```

### 2. Conclusão Imediata

Notifica `complete` imediatamente após emitir todos os valores.

```typescript
import { of } from 'rxjs';

of(1, 2, 3).subscribe({
  next: val => console.log(val),
  complete: () => console.log('Completo!')
});

// Saída: 1, 2, 3, Completo!
```

### 3. Pode Emitir Qualquer Tipo de Valor

Valores de qualquer tipo podem ser emitidos, de tipos primitivos a objetos e arrays.

```typescript
import { of } from 'rxjs';

// Tipos primitivos
of(42, 'olá', true).subscribe(console.log);

// Objetos
of(
  { id: 1, name: 'Alice' },
  { id: 2, name: 'Bob' }
).subscribe(console.log);

// Arrays (emite o array em si como um valor único)
of([1, 2, 3], [4, 5, 6]).subscribe(console.log);
// Saída: [1, 2, 3], [4, 5, 6]
```

### 4. Observable Cold

`of()` é um **Observable Cold**. Cada assinatura inicia uma execução independente.

```typescript
import { of } from 'rxjs';

const values$ = of(1, 2, 3);

// Primeira assinatura
values$.subscribe(val => console.log('Assinante A:', val));

// Segunda assinatura (executada independentemente)
values$.subscribe(val => console.log('Assinante B:', val));

// Saída:
// Assinante A: 1
// Assinante A: 2
// Assinante A: 3
// Assinante B: 1
// Assinante B: 2
// Assinante B: 3
```

> [!NOTE]
> **Características do Observable Cold**:
> - Execução independente é iniciada para cada assinatura
> - Cada assinante recebe seu próprio fluxo de dados
> - Se você precisa compartilhar dados, precisa torná-lo Hot com `share()` etc.
>
> Veja [Observable Cold e Observable Hot](/pt/guide/observables/cold-and-hot-observables) para mais informações.

## Diferença Entre of() e from()

`of()` e `from()` têm comportamento diferente ao lidar com arrays. Este é um ponto comum de confusão.

```typescript
import { of, from } from 'rxjs';

// of() - emite o array como um valor único
of([1, 2, 3]).subscribe(console.log);
// Saída: [1, 2, 3]

// from() - emite cada elemento do array individualmente
from([1, 2, 3]).subscribe(console.log);
// Saída: 1, 2, 3
```

> [!IMPORTANT]
> **Critérios de Uso**:
> - Para emitir o array em si → `of([1, 2, 3])`
> - Para emitir cada elemento de um array separadamente → `from([1, 2, 3])`

## Casos de Uso Prático

### 1. Criação de Dados de Teste e Mock

`of()` é mais frequentemente usada para criar dados mock em código de teste.

```typescript
import { of } from 'rxjs';

// Dados mock de usuário
function getMockUser$() {
  return of({
    id: 1,
    name: 'Usuário de Teste',
    email: 'teste@exemplo.com'
  });
}

// Usar em testes
getMockUser$().subscribe(user => {
  console.log('Usuário:', user.name); // Usuário: Usuário de Teste
});
```

### 2. Fornecendo Valores Padrão

Usado para fornecer valores de fallback em caso de erros ou valores padrão.

```typescript
import { of, throwError } from 'rxjs';
import { catchError } from 'rxjs';

function fetchData(id: number) {
  if (id < 0) {
    return throwError(() => new Error('ID inválido'));
  }
  return of({ id, data: 'alguns dados' });
}

fetchData(-1).pipe(
  catchError((err: unknown) => {
    console.error('Erro:', (err instanceof Error ? err.message : String(err)));
    return of({ id: 0, data: 'dados padrão' }); // Valor padrão
  })
).subscribe(result => console.log(result));
// Saída: Erro: ID inválido
//         { id: 0, data: 'dados padrão' }
```

### 3. Emitir Múltiplos Valores Gradualmente

Usado para executar múltiplas etapas em sequência.

```typescript
import { of } from 'rxjs';
import { concatMap, delay } from 'rxjs';

of('Carregando...', 'Processando...', 'Pronto!').pipe(
  concatMap(message => of(message).pipe(delay(1000)))
).subscribe(console.log);

// Saída (a cada 1 segundo):
// Carregando...
// Processando...
// Pronto!
```

### 4. Retornar Valores em Ramificação Condicional

Usado em combinação com `iif()` e `switchMap()` para retornar valores de acordo com condições.

```typescript
import { of, iif } from 'rxjs';

const isAuthenticated = true;

iif(
  () => isAuthenticated,
  of('Bem-vindo de volta!'),
  of('Por favor, faça login')
).subscribe(console.log);
// Saída: Bem-vindo de volta!
```

## Usando em Pipeline

`of()` é usado como ponto de partida de um pipeline ou para injetar dados ao longo do caminho.

```typescript
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

of(1, 2, 3, 4, 5).pipe(
  filter(n => n % 2 === 0),  // Apenas números pares
  map(n => n * 10)           // Multiplicar por 10
).subscribe(console.log);
// Saída: 20, 40
```

## Erros Comuns

### 1. Passando um Array Diretamente

```typescript
// ❌ Errado - todo o array é emitido como um valor único
of([1, 2, 3]).subscribe(console.log);
// Saída: [1, 2, 3]

// ✅ Correto - use from() para emitir cada elemento separadamente
from([1, 2, 3]).subscribe(console.log);
// Saída: 1, 2, 3

// ✅ Ou use sintaxe de spread
of(...[1, 2, 3]).subscribe(console.log);
// Saída: 1, 2, 3
```

### 2. Confusão com Processamento Assíncrono

Note que `of()` emite sincronamente. Não é processamento assíncrono.

```typescript
// ❌ Isso não se torna assíncrono
of(fetchDataFromAPI()).subscribe(console.log);
// fetchDataFromAPI() executa imediatamente e o objeto Promise é emitido

// ✅ Use from() para fazer stream de uma Promise
from(fetchDataFromAPI()).subscribe(console.log);
```

## Considerações de Desempenho

`of()` é muito leve e tem pouco overhead de desempenho. No entanto, ao emitir grande número de valores, lembre-se do seguinte.

> [!TIP]
> Ao emitir um grande número de valores (milhares ou mais) sequencialmente, considere usar `from()` ou `range()`.

## Funções de Criação Relacionadas

| Função | Diferença | Uso |
|----------|------|----------|
| **[from()](/pt/guide/creation-functions/basic/from)** | Converter de array ou Promise | Fazer stream de iterables ou Promises |
| **range()** | Gerar um intervalo de números | Emitir números consecutivos |
| **EMPTY** | Completar imediatamente sem emitir nada | Quando um stream vazio é necessário |

## Resumo

- `of()` é a Creation Function mais simples que emite os valores especificados em sequência
- Emitida sincronamente na assinatura e completa instantaneamente
- Ideal para dados de teste e criação de mock
- Se um array é passado, o array em si é emitido (diferente de `from()`)
- Use `from()` para processamento assíncrono

## Próximos Passos

- [from() - Converter de Array, Promise, etc.](/pt/guide/creation-functions/basic/from)
- [Funções de Criação de Combinação](/pt/guide/creation-functions/combination/)
- [Retornar para Funções de Criação Básica](/pt/guide/creation-functions/basic/)
