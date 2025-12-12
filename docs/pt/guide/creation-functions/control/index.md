---
description: Esta seção descreve scheduled e using, que são Creation Functions de controle do RxJS. scheduled controla o tempo de execução Observable especificando um scheduler, e using gerencia automaticamente recursos como WebSocket e identificadores de arquivo de acordo com o ciclo de vida Observable. Também pode ser usado para testes e otimização de desempenho.
---

# Creation Functions de Controle

O RxJS fornece Creation Functions para controlar o tempo de execução Observable e o gerenciamento de recursos em detalhes. Esta seção descreve duas funções, `scheduled()` e `using()`, em detalhes.

## O que são Creation Functions de Controle?

Creation Functions de Controle são um conjunto de funções para um controle mais fino do comportamento do Observable. Elas suportam casos de uso avançados, como controle de tempo de execução (scheduler) e gerenciamento de ciclo de vida de recursos.

### Principais Recursos

- **Controle de tempo de execução**: Use scheduler para alternar entre execução síncrona e assíncrona
- **Gerenciamento de recursos**: Liberação automática de recursos de acordo com o ciclo de vida Observable
- **Facilidade de teste**: Alternar entre schedulers para facilitar o teste
- **Otimização de desempenho**: Controlar o tempo de execução para evitar bloqueio de UI

## Lista de Creation Functions de Controle

| Função | Descrição | Usos Principais |
|------|------|---------|
| [scheduled()](/pt/guide/creation-functions/control/scheduled) | Gerar Observable com scheduler especificado | Controle de tempo de execução, teste |
| [using()](/pt/guide/creation-functions/control/using) | Observable com controle de recursos | Gerenciamento de recursos para WebSocket, identificadores de arquivo, etc. |

## Fundamentos de scheduled()

`scheduled()` é uma função que permite especificar explicitamente um scheduler ao gerar um Observable a partir de uma fonte de dados existente (array, Promise, Iterable, etc.).

### Uso Básico

```typescript
import { scheduled, asyncScheduler } from 'rxjs';

// Emitir array de forma assíncrona
const observable$ = scheduled([1, 2, 3], asyncScheduler);

console.log('Iniciar assinatura');
observable$.subscribe({
  next: val => console.log('Valor:', val),
  complete: () => console.log('Completo')
});
console.log('Fim da assinatura');

// Saída:
// Iniciar assinatura
// Fim da assinatura
// Valor: 1
// Valor: 2
// Valor: 3
// Completo
```

> [!NOTE]
> Com `asyncScheduler`, a emissão de valor torna-se assíncrona. Isso permite que o processo de assinatura seja executado sem bloquear o thread principal.

## Fundamentos de using()

`using()` é uma função que cria e libera automaticamente recursos de acordo com o ciclo de vida do Observable. Ela cria um recurso no início de uma assinatura e o libera automaticamente quando a assinatura termina (`complete` ou `unsubscribe`).

### Uso Básico

```typescript
import { using, interval, Subscription, take } from 'rxjs';

const resource$ = using(
  // Fábrica de recursos: executada no início da assinatura
  () => {
    console.log('Recurso criado');
    return new Subscription(() => console.log('Recurso liberado'));
  },
  // Fábrica Observable: criar Observable usando recurso
  () => interval(1000).pipe(take(3))
);

resource$.subscribe({
  next: value => console.log('Valor:', value),
  complete: () => console.log('Completo')
});

// Saída:
// Recurso criado
// Valor: 0
// Valor: 1
// Valor: 2
// Completo
// Recurso liberado
```

> [!IMPORTANT]
> `using()` libera automaticamente recursos no final de uma assinatura, evitando assim vazamentos de memória.

## Comparação: scheduled() vs using()

| Recurso | scheduled() | using() |
|------|-------------|---------|
| Propósito Principal | Controle de tempo de execução | Gerenciamento de ciclo de vida de recursos |
| Scheduler | ✅ Pode especificar explicitamente | ❌ Não pode especificar |
| Gerenciamento de Recursos | ❌ Gerenciamento manual necessário | ✅ Gerenciamento automático |
| Casos de Uso | Teste, otimização de UI | WebSocket, identificadores de arquivo |
| Complexidade | Simples | Um pouco complexo |

## Diretrizes de Uso

### Quando Escolher scheduled()

1. **Deseja controlar o tempo de execução**
   - Deseja mudar processamento síncrono para assíncrono
   - Deseja evitar bloqueio de UI

2. **Precisa de controle de tempo para teste**
   - Combinar com TestScheduler para controlar o tempo
   - Deseja testar processamento assíncrono de forma síncrona

3. **Converter fontes de dados existentes para Observable**
   - Converter Array, Promise, Iterable para Observable
   - Deseja especificar explicitamente um scheduler

### Quando Escolher using()

1. **Liberação automática de recursos é necessária**
   - Gerenciar conexões WebSocket
   - Gerenciamento de identificadores de arquivo
   - Limpeza automática de temporizadores

2. **Deseja prevenir vazamentos de memória**
   - Prevenir esquecimento de liberar recursos
   - Limpeza confiável no final da assinatura

3. **Gerenciamento complexo de recursos**
   - Gerenciar vários recursos de uma vez
   - Gerenciar dependências de recursos

## Exemplos de Uso Prático

### Exemplo de Uso de scheduled()

```typescript
import { scheduled, asyncScheduler, queueScheduler } from 'rxjs';

// Processar grandes quantidades de dados de forma assíncrona (não bloqueia UI)
const largeArray = Array.from({ length: 10000 }, (_, i) => i);
const async$ = scheduled(largeArray, asyncScheduler);

async$.subscribe(value => {
  // Executar processamento pesado aqui
  // UI não é bloqueada
});

// Executar sincronamente em testes
const sync$ = scheduled(largeArray, queueScheduler);
```

### Exemplo de Uso de using()

```typescript
import { using, timer } from 'rxjs';

// Gerenciar automaticamente conexão WebSocket
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    console.log('Conexão WebSocket iniciada');
    return {
      unsubscribe: () => {
        ws.close();
        console.log('Conexão WebSocket encerrada');
      }
    };
  },
  () => timer(0, 1000) // Receber mensagens a cada 1 segundo
);
```

## Tipos de Scheduler (para scheduled())

| Scheduler | Descrição | Casos de Uso |
|---------------|------|---------|
| `queueScheduler` | Execução síncrona (método de fila) | Padrão, processamento síncrono |
| `asyncScheduler` | Execução assíncrona (setTimeout) | Otimização de UI, processamento de longa duração |
| `asapScheduler` | Execução assíncrona mais rápida (Promise) | Processamento assíncrono de alta prioridade |
| `animationFrameScheduler` | Quadro de animação | Animação, renderização de UI |

> [!TIP]
> Para mais informações sobre schedulers, consulte [Tipos de Scheduler](/pt/guide/schedulers/types).

## Perguntas Frequentes

### Q1: Qual é a diferença entre scheduled() e from()?

**A:** `from()` usa o scheduler padrão (síncrono) internamente. `scheduled()` permite que o scheduler seja especificado explicitamente, permitindo assim um controle fino do tempo de execução.

```typescript
import { from, scheduled, asyncScheduler } from 'rxjs';

// from() - executar sincronamente
const sync$ = from([1, 2, 3]);

// scheduled() - executar de forma assíncrona
const async$ = scheduled([1, 2, 3], asyncScheduler);
```

### Q2: Quando devo usar using()?

**A:** Use quando quiser evitar esquecer de liberar recursos. É especialmente útil nos seguintes casos:
- Conexões de rede como WebSocket, EventSource, etc.
- Identificadores de arquivo, conexões de banco de dados
- Processos que requerem `clearInterval()` ou `clearTimeout()` manual

### Q3: Por que scheduled() é mais fácil de testar?

**A:** TestScheduler permite controlar virtualmente a passagem do tempo. Processos assíncronos podem ser testados de forma síncrona, reduzindo muito o tempo de execução do teste.

```typescript
import { TestScheduler } from 'rxjs/testing';
import { scheduled, asyncScheduler } from 'rxjs';

const testScheduler = new TestScheduler((actual, expected) => {
  expect(actual).toEqual(expected);
});

testScheduler.run(({ expectObservable }) => {
  const source$ = scheduled([1, 2, 3], testScheduler);
  expectObservable(source$).toBe('(abc|)', { a: 1, b: 2, c: 3 });
});
```

## Melhores Práticas

### 1. Evitar Bloqueio de UI com scheduled()

```typescript
// ❌ Exemplo ruim: Processar grandes quantidades de dados sincronamente
from(largeArray).subscribe(processHeavyTask);

// ✅ Exemplo bom: Processamento assíncrono com asyncScheduler
scheduled(largeArray, asyncScheduler).subscribe(processHeavyTask);
```

### 2. Garantir Liberação de Recursos com using()

```typescript
// ❌ Exemplo ruim: Gerenciamento manual de recursos
const ws = new WebSocket('wss://example.com');
const source$ = interval(1000);
source$.subscribe(() => ws.send('ping'));
// Vazamento de recurso se unsubscribe esquecido

// ✅ Exemplo bom: Gerenciamento automático com using()
const websocket$ = using(
  () => {
    const ws = new WebSocket('wss://example.com');
    return { unsubscribe: () => ws.close() };
  },
  () => interval(1000).pipe(tap(() => ws.send('ping')))
);
```

### 3. Usar Scheduler Apropriado para Teste

```typescript
// ✅ Exemplo bom: TestScheduler para teste
const testScheduler = new TestScheduler(...);
const source$ = scheduled([1, 2, 3], testScheduler);

// ✅ Exemplo bom: asyncScheduler para produção
const source$ = scheduled([1, 2, 3], asyncScheduler);
```

## Resumo

Creation Functions de Controle são funções avançadas para ajustar finamente o comportamento do Observable.

**scheduled():**
- Controla explicitamente o tempo de execução (síncrono/assíncrono)
- Útil para controle de tempo em teste
- Eficaz para evitar bloqueio de UI

**using():**
- Gerenciamento automático do ciclo de vida de recursos
- Previne vazamentos de memória
- Ideal para gerenciar conexões como WebSocket

Usado adequadamente, você pode construir aplicações RxJS mais robustas e performáticas.

## Próximos Passos

Para uso detalhado de cada função, consulte as seguintes páginas:

- [scheduled() em detalhes](/pt/guide/creation-functions/control/scheduled) - Gerar Observable com scheduler
- [using() em detalhes](/pt/guide/creation-functions/control/using) - Observable com controle de recursos

## Recursos de Referência

- [Documentação Oficial RxJS - scheduled](https://rxjs.dev/api/index/function/scheduled)
- [Documentação Oficial RxJS - using](https://rxjs.dev/api/index/function/using)
- [Documentação Oficial RxJS - Scheduler](https://rxjs.dev/guide/scheduler)
- [Tipos de Scheduler](/pt/guide/schedulers/types)
