---
description: Esta seção descreve as Creation Functions que combinam múltiplos Observables em um único Observable, e ensina como usar concat, merge, combineLatest, zip e forkJoin, além de exemplos práticos.
---

# Creation Functions de Combinação

Estas são as principais Creation Functions para combinar múltiplos Observables em um único Observable.

## O que são Creation Functions de Combinação?

As Creation Functions de Combinação recebem múltiplos Observables e os combinam em um único fluxo Observable. O momento e a ordem em que os valores são emitidos dependem do método de combinação.

A tabela abaixo mostra as características de cada Creation Function e como usá-las.

## Principais Creation Functions de Combinação

| Função | Descrição | Casos de Uso |
|----------|------|-------------|
| **[concat](/pt/guide/creation-functions/combination/concat)** | Combinação sequencial (próximo inicia após o anterior completar) | Processamento passo a passo |
| **[merge](/pt/guide/creation-functions/combination/merge)** | Combinação paralela (inscreve simultaneamente, emite na ordem de emissão) | Integração de múltiplos eventos |
| **[combineLatest](/pt/guide/creation-functions/combination/combineLatest)** | Combina valores mais recentes | Sincronização de entrada de formulário |
| **[zip](/pt/guide/creation-functions/combination/zip)** | Emparelha valores correspondentes | Correspondência de requisições com respostas |
| **[forkJoin](/pt/guide/creation-functions/combination/forkJoin)** | Aguarda todos completarem e combina valores finais | Aguardar conclusão de chamadas de API paralelas |

## Critérios de Uso

A seleção das Creation Functions de Combinação é determinada a partir das seguintes perspectivas:

### 1. Momento de Execução

- **Execução sequencial**: `concat` - Inicia o próximo após o Observable anterior completar
- **Execução paralela**: `merge`, `combineLatest`, `zip`, `forkJoin` - Inscreve em todos os Observables simultaneamente

### 2. Como Emitir Valores

- **Emitir todos os valores**: `concat`, `merge` - Emite todos os valores de cada Observable
- **Combinar valores mais recentes**: `combineLatest` - Combina todos os valores mais recentes sempre que um deles emite
- **Emparelhar valores correspondentes**: `zip` - Emparelha valores de posições correspondentes em cada Observable e emite
- **Apenas valores finais**: `forkJoin` - Emite cada valor final como um array quando todos os Observables completam

### 3. Momento de Conclusão

- **Após todos completarem**: `concat`, `forkJoin` - Aguarda até que todos os Observables tenham completado
- **Completa com o fluxo mais curto**: `zip` - Completa quando qualquer um completa, pois os valores restantes não podem formar pares
- **Não completa**: `merge`, `combineLatest` - Se um completa enquanto o outro continua, não completará

## Convertendo Cold para Hot

Conforme mostrado na tabela acima, **todas as Creation Functions de Combinação geram Observables Cold**. Cada inscrição inicia uma execução independente.

No entanto, você pode **converter um Observable Cold para Hot** usando um operador multicast (`share()`, `shareReplay()`, `publish()`, etc.).

### Exemplo Prático: Compartilhando Requisições HTTP

```typescript
import { merge, interval } from 'rxjs';
import { map, take, share } from 'rxjs';

// ❄️ Cold - Requisições HTTP independentes para cada inscrição
const coldApi$ = merge(
  interval(1000).pipe(map(() => 'Fonte A'), take(3)),
  interval(1500).pipe(map(() => 'Fonte B'), take(2))
);

coldApi$.subscribe(val => console.log('Assinante 1:', val));
coldApi$.subscribe(val => console.log('Assinante 2:', val));
// → Cada assinante executa intervalos independentes (2x requisições)

// 🔥 Hot - Compartilha execução entre assinantes
const hotApi$ = merge(
  interval(1000).pipe(map(() => 'Fonte A'), take(3)),
  interval(1500).pipe(map(() => 'Fonte B'), take(2))
).pipe(share());

hotApi$.subscribe(val => console.log('Assinante 1:', val));
hotApi$.subscribe(val => console.log('Assinante 2:', val));
// → Compartilha um intervalo (requisita apenas uma vez)
```

> [!TIP]
> **Casos em que a conversão Hot é necessária**:
> - Múltiplos componentes compartilham os mesmos resultados de API
> - Use `forkJoin` para usar os resultados de requisições paralelas em múltiplos locais
> - Gerencie estado com `combineLatest` e distribua para múltiplos assinantes
>
> Para mais informações, consulte [Criação Básica - Convertendo Cold para Hot](/pt/guide/creation-functions/basic/#converting-cold-to-hot).

## Correspondência com Pipeable Operator

Para as Creation Functions de Combinação, existe um Pipeable Operator correspondente. Quando usado em um pipeline, o operador do tipo `~With` é utilizado.

| Creation Function | Pipeable Operator |
|-------------------|-------------------|
| `concat(a$, b$)` | `a$.pipe(concatWith(b$))` |
| `merge(a$, b$)` | `a$.pipe(mergeWith(b$))` |
| `zip(a$, b$)` | `a$.pipe(zipWith(b$))` |
| `combineLatest([a$, b$])` | `a$.pipe(combineLatestWith(b$))` |

## Próximos Passos

Para aprender o comportamento detalhado e exemplos práticos de cada Creation Function, clique nos links da tabela acima.

Além disso, ao aprender sobre [Creation Functions de Seleção/Partição](/pt/guide/creation-functions/selection/) e [Creation Functions Condicionais](/pt/guide/creation-functions/conditional/), você pode compreender o quadro completo das Creation Functions.
