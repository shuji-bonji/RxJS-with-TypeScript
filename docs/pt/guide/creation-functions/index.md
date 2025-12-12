---
description: Uma explicação abrangente das Creation Functions do RxJS (funções de criação de Observable), incluindo diferenças em relação ao Pipeable Operator, uso básico e sete categorias (criação básica, geração de loop, comunicação HTTP, combinação, seleção/partição, ramificação condicional e sistemas de controle).
---

# Creation Functions

No RxJS, existem duas formas diferentes: **Creation Functions** para criar Observables e **Pipeable Operators** para converter Observables existentes.

Esta página descreve os conceitos básicos das Creation Functions e as sete categorias principais.

## O que são Creation Functions?

**Creation Functions** são funções para criar novos Observables.

```typescript
import { of, from, interval } from 'rxjs';

// Usando como Creation Functions
const obs1$ = of(1, 2, 3);
const obs2$ = from([4, 5, 6]);
const obs3$ = interval(1000);
```

Elas são importadas diretamente do pacote `rxjs` e chamadas como funções para criar Observables.

## Diferença em relação ao Pipeable Operator

Creation Functions e Pipeable Operators têm usos e aplicações diferentes. Veja a tabela abaixo para ver as diferenças entre eles.

| Característica | Creation Function | Pipeable Operator |
|------|-------------------|-------------------|
| **Objetivo** | Criar novo Observable | Transformar Observable existente |
| **Importar de** | `rxjs` | `rxjs/operators` |
| **Uso** | Chamar diretamente como função | Usar dentro de `.pipe()` |
| **Exemplo** | `concat(obs1$, obs2$)` | `obs1$.pipe(concatWith(obs2$))` |

### Exemplo de Creation Function

Creation Functions são usadas para combinar diretamente múltiplos Observables.

```typescript
import { concat, of } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Usando como Creation Function
concat(obs1$, obs2$).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5, 6
```

### Exemplo de Pipeable Operator

O Pipeable Operator é usado para adicionar um processo de conversão a um Observable existente.

```typescript
import { of } from 'rxjs';
import { concatWith } from 'rxjs';

const obs1$ = of(1, 2, 3);
const obs2$ = of(4, 5, 6);

// Usando como Pipeable Operator
obs1$.pipe(
  concatWith(obs2$)
).subscribe(console.log);
// Saída: 1, 2, 3, 4, 5, 6
```

## Critérios de Uso

A escolha entre Creation Function e Pipeable Operator é determinada pelos seguintes critérios.

### Quando usar Creation Function

A Creation Function é adequada quando múltiplos Observables devem ser operados no mesmo nível ou quando um Observable deve ser criado do zero.

- **Ao combinar múltiplos Observables no mesmo nível**
  ```typescript
  concat(obs1$, obs2$, obs3$)
  merge(click$, hover$, scroll$)
  ```

- **Ao criar um Observable do zero**
  ```typescript
  of(1, 2, 3)
  from([1, 2, 3])
  interval(1000)
  ```

### Quando usar Pipeable Operator

O Pipeable Operator é adequado para adicionar processamento a um Observable existente ou para encadear múltiplas operações juntas.

- **Ao adicionar operações a um Observable existente**
  ```typescript
  obs1$.pipe(
    map(x => x * 2),
    concatWith(obs2$),
    filter(x => x > 5)
  )
  ```

- **Ao encadear múltiplas operações como um pipeline**

## Categorias de Creation Functions

Neste capítulo, as Creation Functions são divididas em sete categorias.

### Lista de Todas as Categorias

Na tabela abaixo, você pode ver todas as categorias e as funções que elas contêm. Clique em cada nome de função para ir à página de detalhes.

| Categoria | Descrição | Funções Principais | Casos de Uso Típicos |
|---------|------|-----------|-------------------|
| **[Criação Básica](/pt/guide/creation-functions/basic/)** | Funções mais básicas e frequentemente usadas. Criar Observables baseados em dados, arrays, eventos e tempo | [of](/pt/guide/creation-functions/basic/of), [from](/pt/guide/creation-functions/basic/from), [fromEvent](/pt/guide/creation-functions/basic/fromEvent), [interval](/pt/guide/creation-functions/basic/interval), [timer](/pt/guide/creation-functions/basic/timer) | Testes com valores fixos, streaming de dados existentes, manipulação de eventos DOM, polling, execução atrasada |
| **[Geração de Loop](/pt/guide/creation-functions/loop/)** | Expressar processamento de loop como declarações for/while em Observable | [range](/pt/guide/creation-functions/loop/range), [generate](/pt/guide/creation-functions/loop/generate) | Geração de números sequenciais, processamento em lote, transições de estado complexas, cálculos matemáticos |
| **[Comunicação HTTP](/pt/guide/creation-functions/http-communication/)** | Manipular comunicação HTTP como Observable | [ajax](/pt/guide/creation-functions/http-communication/ajax), [fromFetch](/pt/guide/creation-functions/http-communication/fromFetch) | Comunicação HTTP baseada em XMLHttpRequest, comunicação HTTP baseada em Fetch API, chamadas de API REST |
| **[Combinação](/pt/guide/creation-functions/combination/)** | Combinar múltiplos Observables em um. O timing e a ordem de emissão diferem dependendo do método de combinação | [concat](/pt/guide/creation-functions/combination/concat), [merge](/pt/guide/creation-functions/combination/merge), [combineLatest](/pt/guide/creation-functions/combination/combineLatest), [zip](/pt/guide/creation-functions/combination/zip), [forkJoin](/pt/guide/creation-functions/combination/forkJoin) | Processamento passo a passo, integração de múltiplos eventos, sincronização de entradas de formulário, espera pela conclusão de chamadas de API paralelas |
| **[Seleção/Partição](/pt/guide/creation-functions/selection/)** | Selecionar um de múltiplos Observables ou particionar um Observable em múltiplos | [race](/pt/guide/creation-functions/selection/race), [partition](/pt/guide/creation-functions/selection/partition) | Competição entre múltiplas fontes de dados, ramificação de sucesso/falha |
| **[Condicional](/pt/guide/creation-functions/conditional/)** | Selecionar Observable com base em condições ou gerar dinamicamente no momento da assinatura | [iif](/pt/guide/creation-functions/conditional/iif), [defer](/pt/guide/creation-functions/conditional/defer) | Ramificação de processamento baseada em status de login, criação dinâmica de Observable, avaliação preguiçosa |
| **[Controle](/pt/guide/creation-functions/control/)** | Controlar o timing de execução do Observable e gerenciamento de recursos | [scheduled](/pt/guide/creation-functions/control/scheduled), [using](/pt/guide/creation-functions/control/using) | Controle de timing de execução com scheduler, gerenciamento de ciclo de vida de recursos, prevenção de vazamento de memória |

> [!TIP]
> **Ordem de Aprendizado**
>
> Recomendamos que iniciantes aprendam na seguinte ordem:
> 1. **Criação Básica** - Funções fundamentais do RxJS
> 2. **Combinação** - Fundamentos de manipulação de múltiplos streams
> 3. **Comunicação HTTP** - Integração prática de API
> 4. Outras categorias - Aprender conforme necessário

## Correspondência com Pipeable Operator

Muitas Creation Functions têm um Pipeable Operator correspondente. Quando usado em um pipeline, use um operador da família `~With`.

| Creation Function | Pipeable Operator | Notas |
|-------------------|-------------------|------|
| `concat(a$, b$)` | `a$.pipe(`**[concatWith](/pt/guide/operators/combination/concatWith)**`(b$))` | RxJS 7+ |
| `merge(a$, b$)` | `a$.pipe(`**[mergeWith](/pt/guide/operators/combination/mergeWith)**`(b$))` | RxJS 7+ |
| `zip(a$, b$)` | `a$.pipe(`**[zipWith](/pt/guide/operators/combination/zipWith)**`(b$))` | RxJS 7+ |
| `combineLatest([a$, b$])` | `a$.pipe(`**[combineLatestWith](/pt/guide/operators/combination/combineLatestWith)**`(b$))` | RxJS 7+ |
| `race(a$, b$)` | `a$.pipe(`**[raceWith](/pt/guide/operators/combination/raceWith)**`(b$))` | RxJS 7+ |

> [!NOTE]
> Desde o RxJS 7, **[concatWith](/pt/guide/operators/combination/concatWith)**, **[mergeWith](/pt/guide/operators/combination/mergeWith)**, **[zipWith](/pt/guide/operators/combination/zipWith)**, **[combineLatestWith](/pt/guide/operators/combination/combineLatestWith)**, **[raceWith](/pt/guide/operators/combination/raceWith)** e outros operadores do tipo `~With` foram adicionados, facilitando o uso como Pipeable Operator.

## Qual Devo Usar?

A escolha entre Creation Function e Pipeable Operator depende do contexto.

### Creation Function é Recomendada

Se múltiplos Observables devem ser operados no mesmo nível, a Creation Function simplificará o código.

```typescript
// ✅ Combinar múltiplos Observables no mesmo nível
const combined$ = merge(
  fromEvent(button1, 'click'),
  fromEvent(button2, 'click'),
  fromEvent(button3, 'click')
);
```

### Pipeable Operator é Recomendado

Ao adicionar operações como parte de um pipeline, use Pipeable Operator para esclarecer o fluxo de processamento.

```typescript
// ✅ Combinar como parte do pipeline
const result$ = source$.pipe(
  map(x => x * 2),
  mergeWith(other$),
  filter(x => x > 10)
);
```

## Resumo

- **Creation Functions**: Funções para criar e combinar Observables
- **Pipeable Operators**: Funções para converter Observables existentes
- Creation Functions se dividem em 7 categorias:
  1. **Criação Básica**: Criar Observables baseados em dados, arrays, eventos e tempo
  2. **Geração de Loop**: Expressar processamento iterativo em Observable
  3. **Comunicação HTTP**: Manipular comunicação HTTP como Observable
  4. **Combinação**: Combinar múltiplos em um
  5. **Seleção/Partição**: Selecionar ou particionar
  6. **Condicional**: Gerar dinamicamente de acordo com condições
  7. **Controle**: Controlar timing de execução e gerenciamento de recursos
- Use operadores Pipeable da família `~With` em pipelines
- Cada categoria contém múltiplas funções e pode ser usada de maneiras diferentes dependendo da aplicação

## Próximos Passos

Para aprender mais sobre cada categoria, siga os links abaixo:

1. **[Funções de Criação Básica](/pt/guide/creation-functions/basic/)** - of, from, fromEvent, interval, timer
2. **[Funções de Geração de Loop](/pt/guide/creation-functions/loop/)** - range, generate
3. **[Funções de Comunicação HTTP](/pt/guide/creation-functions/http-communication/)** - ajax, fromFetch
4. **[Funções de Combinação](/pt/guide/creation-functions/combination/)** - concat, merge, combineLatest, zip, forkJoin
5. **[Funções de Seleção/Partição](/pt/guide/creation-functions/selection/)** - race, partition
6. **[Funções Condicionais](/pt/guide/creation-functions/conditional/)** - iif, defer
7. **[Funções de Controle](/pt/guide/creation-functions/control/)** - scheduled, using

Em cada página, você aprenderá mais sobre como as Creation Functions funcionam e exemplos práticos.

## Recursos de Referência

- [Documentação Oficial do RxJS - Creation Functions](https://rxjs.dev/guide/operators#creation-operators-list)
- [Learn RxJS - Creation Operators](https://www.learnrxjs.io/learn-rxjs/operators/creation)
