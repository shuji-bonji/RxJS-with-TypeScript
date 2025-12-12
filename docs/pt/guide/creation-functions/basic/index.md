---
description: Esta seção aborda Creation Functions para criação básica de Observable, usando of, from, fromEvent, interval e timer para criar Observables de uma variedade de fontes de dados, incluindo valores únicos, arrays, Promises, eventos e timers. Este é um conceito importante que pode ser implementado com segurança de tipo TypeScript e é a base do RxJS.
---

# Funções de Criação Básica

As Creation Functions mais básicas e frequentemente usadas. Crie facilmente Observables baseados em dados, arrays, eventos e tempo.

## O que são Funções de Criação Básica?

Funções de Criação Básica são funções para criar um único Observable de uma variedade de fontes de dados. Elas são o conjunto de funções mais fundamental no RxJS e são usadas em quase todo código RxJS.

Por favor, revise a tabela abaixo para ver as características e uso de cada Creation Function.

## Principais Funções de Criação Básica

| Função | Descrição | Casos de Uso |
|----------|------|-------------|
| **[of](/pt/guide/creation-functions/basic/of)** | Emitir valores especificados em sequência | Testes com valores fixos, criação de mocks |
| **[from](/pt/guide/creation-functions/basic/from)** | Converter de array, Promise, etc. | Streaming de dados existentes |
| **[fromEvent](/pt/guide/creation-functions/basic/fromEvent)** | Converter eventos para Observable | Eventos DOM, Node.js EventEmitter |
| **[interval](/pt/guide/creation-functions/basic/interval)** | Emitir continuamente em intervalos especificados | Polling, execução periódica |
| **[timer](/pt/guide/creation-functions/basic/timer)** | Começar a emitir após um atraso | Execução atrasada, timeout |

## Critérios de Uso

A escolha das Funções de Criação Básica é determinada pelo tipo de fonte de dados.

### 1. Tipo de Dados

- **Valores estáticos**: `of()` - Cria um Observable especificando o valor diretamente
- **Array ou Iterable**: `from()` - Converter uma coleção existente em um stream
- **Promise**: `from()` - Converter processamento assíncrono em Observable
- **Evento**: `fromEvent()` - Converte um event listener em um Observable
- **Baseado em tempo**: `interval()`, `timer()` - Emitir valores baseados na passagem do tempo

### 2. Timing da Emissão

- **Emitir imediatamente**: `of()`, `from()` - Começar a emitir valores na assinatura
- **Na ocorrência do evento**: `fromEvent()` - Emitir sempre que um evento ocorrer
- **Emitir periodicamente**: `interval()` - Emitir continuamente em intervalos regulares
- **Emitir após um atraso**: `timer()` - Começar a emitir após um tempo especificado

### 3. Timing da Conclusão

- **Completar imediatamente**: `of()`, `from()` - Completar após todos os valores serem emitidos
- **Nunca completar**: `fromEvent()`, `interval()` - Continuar até unsubscribe
- **Emitir uma vez e completar**: `timer(delay)` - Completar após emitir um valor

## Exemplos de Uso Prático

### of() - Testes com Valores Fixos

```typescript
import { of } from 'rxjs';

// Criar dados de teste
const mockUser$ = of({ id: 1, name: 'Usuário de Teste' });

mockUser$.subscribe(user => console.log(user));
// Saída: { id: 1, name: 'Usuário de Teste' }
```

### from() - Streaming de um Array

```typescript
import { from } from 'rxjs';
import { map } from 'rxjs';

const numbers$ = from([1, 2, 3, 4, 5]);

numbers$.pipe(
  map(n => n * 2)
).subscribe(console.log);
// Saída: 2, 4, 6, 8, 10
```

### fromEvent() - Evento de Clique

```typescript
import { fromEvent } from 'rxjs';

const clicks$ = fromEvent(document, 'click');

clicks$.subscribe(() => console.log('Botão clicado!'));
```

### interval() - Polling

```typescript
import { interval } from 'rxjs';
import { switchMap } from 'rxjs';

// Fazer polling da API a cada 5 segundos
interval(5000).pipe(
  switchMap(() => fetchData())
).subscribe(data => console.log('Atualizado:', data));
```

### timer() - Execução Atrasada

```typescript
import { timer } from 'rxjs';

// Executar após 3 segundos
timer(3000).subscribe(() => console.log('3 segundos se passaram'));
```

## Cuidado com Vazamentos de Memória

O cancelamento adequado da assinatura é importante ao usar as Funções de Criação Básica.

> [!WARNING]
> `fromEvent()`, `interval()`, e `timer(delay, period)` periódico não completarão e sempre devem ter `unsubscribe()` ou cancelamento automático da assinatura com `takeUntil()` ou similar quando o componente for destruído.
>
> Nota: `timer(delay)` sem um segundo argumento completará automaticamente após emitir uma vez.

```typescript
import { fromEvent, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class MyComponent {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    fromEvent(window, 'resize').pipe(
      takeUntil(this.destroy$)
    ).subscribe(() => console.log('Janela redimensionada'));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

## Convertendo Cold para Hot

Como mostrado na tabela acima, **todas as Funções de Criação Básica geram Observables Cold**. Cada assinatura inicia uma execução independente.

No entanto, você pode **converter um Observable Cold em um Observable Hot** usando os seguintes operadores de multicast.

### Condições e Operadores para Conversão em Hot

| Operador | Comportamento | Casos de Uso |
|-------------|------|-------------|
| **share()** | Multicast + conectar/desconectar automaticamente | Compartilhar requisições HTTP entre múltiplos assinantes |
| **shareReplay(n)** | Armazenar em cache os últimos n valores e entregar aos novos assinantes | Armazenar em cache respostas de API |
| **publish() + connect()** | Iniciar manualmente o multicast | Iniciar execução quando os assinantes estiverem prontos |
| **multicast(subject)** | Multicast com Subject personalizado | Quando controle avançado é necessário |

### Exemplo Prático

```typescript
import { interval } from 'rxjs';
import { take, share } from 'rxjs';

// ❄️ Cold - Timer independente para cada assinatura
const cold$ = interval(1000).pipe(take(3));

cold$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  cold$.subscribe(val => console.log('B:', val));
}, 1500);

// Saída:
// A: 0 (após 0s)
// A: 1 (após 1s)
// B: 0 (após 1.5s) ← B começa independentemente de 0
// A: 2 (após 2s)
// B: 1 (após 2.5s)

// 🔥 Hot - Compartilha timer entre assinantes
const hot$ = interval(1000).pipe(take(3), share());

hot$.subscribe(val => console.log('A:', val));
setTimeout(() => {
  hot$.subscribe(val => console.log('B:', val));
}, 1500);

// Saída:
// A: 0 (após 0s)
// A: 1 (após 1s)
// A: 2, B: 2 (após 2s) ← B se junta no meio, recebe o mesmo valor
```

> [!TIP]
> **Casos onde conversão Hot é necessária**:
> - Deseja compartilhar uma requisição HTTP entre múltiplos assinantes
> - Deseja manter apenas uma conexão WebSocket ou servidor
> - Deseja usar os resultados de cálculos de alto custo em múltiplos locais
>
> Para mais informações, veja o capítulo **Subject e Multicast** (Capítulo 5).

## Relação com Pipeable Operator

Não há Pipeable Operator correspondente diretamente às Funções de Criação Básica. Elas são sempre usadas como Creation Functions.

No entanto, elas são usadas em combinação com Pipeable Operators no seguinte padrão:

```typescript
import { fromEvent } from 'rxjs';
import { debounceTime, switchMap } from 'rxjs';

// Entrada do usuário → Esperar 300ms → Chamada de API
fromEvent(input, 'input').pipe(
  debounceTime(300),
  switchMap(event => fetchSuggestions(event.target.value))
).subscribe(suggestions => console.log(suggestions));
```

## Próximos Passos

Para aprender mais sobre como cada Creation Function funciona e exemplos práticos, clique nos links da tabela acima.

Além disso, ao aprender [Funções de Criação de Combinação](/pt/guide/creation-functions/combination/), [Funções de Criação de Seleção/Partição](/pt/guide/creation-functions/selection/), e [Funções de Criação Condicional](/pt/guide/creation-functions/conditional/), você pode entender o quadro completo das Creation Functions.
