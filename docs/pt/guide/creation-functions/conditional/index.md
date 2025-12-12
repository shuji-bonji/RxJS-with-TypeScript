---
description: Esta seção descreve Creation Functions que selecionam e criam Observables com base em condições. Aprenda como usar iif e defer, bem como exemplos práticos.
---

# Creation Functions Condicionais

Creation Functions selecionam um Observable com base em uma condição ou geram dinamicamente um Observable ao assinar.

## O que são Creation Functions Condicionais?

Creation Functions Condicionais têm as seguintes funções:

- **Seleção Condicional**: Selecionar diferentes Observables de acordo com as condições
- **Geração Atrasada**: Criar dinamicamente um Observable no momento da assinatura

Ao contrário de outras Creation Functions, que criam e combinam Observables estaticamente, essas podem alterar seu comportamento com base em **condições e estados de tempo de execução**.

> [!NOTE]
> Embora `iif` e `defer` tenham sido previamente classificados como "operadores condicionais", eles são **Creation Functions** (funções de criação Observable), não Pipeable Operators.

## Principais Creation Functions Condicionais

| Função | Descrição | Casos de Uso |
|----------|------|-------------|
| **[iif](/pt/guide/creation-functions/conditional/iif)** | Selecionar um de dois Observables com base em uma condição | Ramificação de processamento com base no status de login |
| **[defer](/pt/guide/creation-functions/conditional/defer)** | Atrasar a geração de Observable no momento da assinatura | Criação dinâmica de Observable |

## Critérios de Uso

### iif - Duas Ramificações Baseadas em Condição

`iif` seleciona um de dois Observables dependendo do resultado de uma função condicional. A condição é avaliada **no momento da assinatura**.

**Sintaxe**:
```typescript
iif(
  () => condition,  // Função de condição (avaliada no momento da assinatura)
  trueObservable,   // Observable se verdadeiro
  falseObservable   // Observable se falso
)
```

**Casos de Uso**:
- Ramificação de processamento com base no status de login
- Alternância de processamento com base na existência de cache
- Mudança de comportamento por variáveis de ambiente

```typescript
import { iif, of } from 'rxjs';

const isAuthenticated = () => Math.random() > 0.5;

const data$ = iif(
  isAuthenticated,
  of('Dados autenticados'),
  of('Dados públicos')
);

data$.subscribe(console.log);
// Saída: 'Dados autenticados' ou 'Dados públicos' (dependendo da condição no momento da assinatura)
```

### defer - Geração Atrasada no Momento da Assinatura

`defer` gera um Observable cada vez que ocorre uma assinatura. Isso permite que o Observable mude seu comportamento com base em seu estado no momento da assinatura.

**Sintaxe**:
```typescript
defer(() => {
  // Executado no momento da assinatura
  return someObservable;
})
```

**Casos de Uso**:
- Gerar Observable refletindo o estado mais recente no momento da assinatura
- Gerar um valor aleatório diferente cada vez
- Executar processamento diferente para cada assinatura

```typescript
import { defer, of } from 'rxjs';

// Obter hora atual na assinatura
const timestamp$ = defer(() => of(new Date().toISOString()));

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Primeiro:', time));
}, 1000);

setTimeout(() => {
  timestamp$.subscribe(time => console.log('Segundo:', time));
}, 2000);

// Saída:
// Primeiro: 2024-10-21T01:00:00.000Z
// Segundo: 2024-10-21T01:00:01.000Z
// ※Tempos diferentes são exibidos porque os horários de assinatura diferem
```

## Diferença Entre iif e defer

| Recurso | iif | defer |
|------|-----|-------|
| **Escolha** | Selecionar de dois Observables | Gerar qualquer Observable |
| **Temporização de Avaliação** | Avaliar condição no momento da assinatura | Executar função no momento da assinatura |
| **Propósito** | Ramificação condicional | Geração dinâmica |

## Usando em Pipeline

Creation Functions Condicionais podem ser usadas em combinação com outros operadores.

```typescript
import { defer, of } from 'rxjs';
import { switchMap } from 'rxjs';

// Obter informações do usuário do ID do usuário
const userId$ = of(123);

userId$.pipe(
  switchMap(id =>
    defer(() => {
      // Verificar cache mais recente no momento da assinatura
      const cached = cache.get(id);
      return cached ? of(cached) : fetchUser(id);
    })
  )
).subscribe(console.log);
```

## Convertendo Cold para Hot

Conforme mostrado na tabela acima, **todas as Creation Functions Condicionais geram Observables Cold**. Avaliações condicionais e funções de geração são executadas cada vez que uma assinatura é feita.

Você pode converter um Observable Cold em um Observable Hot usando operadores multicast (`share()`, `shareReplay()`, etc.).

### Exemplo Prático: Compartilhando Resultados de Ramificação Condicional

```typescript
import { iif, of, interval } from 'rxjs';
import { take, share } from 'rxjs';

const condition = () => Math.random() > 0.5;

// ❄️ Cold - Reavaliar condição para cada assinatura
const coldIif$ = iif(
  condition,
  of('Condição é verdadeira'),
  interval(1000).pipe(take(3))
);

coldIif$.subscribe(val => console.log('Assinante 1:', val));
coldIif$.subscribe(val => console.log('Assinante 2:', val));
// → Cada assinante avalia independentemente a condição (possibilidade de resultados diferentes)

// 🔥 Hot - Compartilhar resultados de avaliação de condição entre assinantes
const hotIif$ = iif(
  condition,
  of('Condição é verdadeira'),
  interval(1000).pipe(take(3))
).pipe(share());

hotIif$.subscribe(val => console.log('Assinante 1:', val));
hotIif$.subscribe(val => console.log('Assinante 2:', val));
// → Condição avaliada apenas uma vez, resultados compartilhados
```

> [!TIP]
> Para mais informações, consulte [Criação Básica - Convertendo Cold para Hot](/pt/guide/creation-functions/basic/#converting-cold-to-hot).

## Próximos Passos

Para aprender o comportamento detalhado e exemplos práticos de cada Creation Function, clique nos links da tabela acima.

Além disso, ao aprender [Creation Functions de Combinação](/pt/guide/creation-functions/combination/) e [Creation Functions de Seleção/Partição](/pt/guide/creation-functions/selection/), você pode entender o panorama geral das Creation Functions.
