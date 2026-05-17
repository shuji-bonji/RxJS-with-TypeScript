---
description: Esta página apresenta seis cenários comuns de depuração para RxJS, incluindo valores que não fluem, valores diferentes dos esperados, subscription que não completa, vazamentos de memória, erros perdidos e rastreamento de retry, juntamente com problemas práticos e soluções.
---

# Cenários Comuns de Depuração

Problemas típicos encontrados no desenvolvimento RxJS e suas soluções são descritos com exemplos de código concretos.

## Cenário 1: Valores não fluem

- **Sintoma**: Eu faço `subscribe` e nem um único valor é emitido.


### Causa 1: Você esqueceu de fazer subscribe ao Cold Observable.

Cold Observable não será executado até que seja subscrito.

```ts
import { interval } from 'rxjs';
import { map } from 'rxjs';

// ❌ Nada é executado porque não há subscription
const numbers$ = interval(1000).pipe(
  map(x => {
    console.log('Esta linha não é executada');
    return x * 2;
  })
);

// ✅ Executado ao fazer subscribe
numbers$.subscribe(value => console.log('Valor:', value));
```

### Causa 2: Subject Completado

Uma vez que um Subject é completado, ele não receberá valores em subscriptions subsequentes.

```ts
import { Subject } from 'rxjs';

const subject = new Subject<number>();

subject.complete(); // Completar

// ❌ Subscription após completar não recebe valor
subject.subscribe(value => console.log('Esta linha não é executada'));

// ✅ Subscribe antes de completar
const subject2 = new Subject<number>();
subject2.subscribe(value => console.log('Valor:', value));
subject2.next(1); // Valor: 1
subject2.complete();
```

### Causa 3: Filtragem em condições erradas

Condições de filtragem podem ser muito estritas e excluir todos os valores.

```ts
import { of } from 'rxjs';
import { filter, tap } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('Antes do filter:', value)),
    filter(x => x > 10), // Todos excluídos
    tap(value => console.log('Após filter:', value)) // Esta linha não é executada
  )
  .subscribe({
    next: value => console.log('Valor final:', value),
    complete: () => console.log('Complete (sem valor)')
  });

// Saída:
// Antes do filter: 1
// Antes do filter: 2
// Antes do filter: 3
// Antes do filter: 4
// Antes do filter: 5
// Complete (sem valor)
```

### Técnicas de Depuração

Use o operador `tap` para ver quais valores estão fluindo em cada etapa.

```ts
import { of, EMPTY } from 'rxjs';
import { filter, tap, defaultIfEmpty } from 'rxjs';

of(1, 2, 3, 4, 5)
  .pipe(
    tap(value => console.log('🔵 Entrada:', value)),
    filter(x => x > 10),
    tap(value => console.log('🟢 Passou no filter:', value)),
    defaultIfEmpty('Sem valor') // Padrão se não houver valor
  )
  .subscribe(value => console.log('✅ Saída:', value));

// Saída:
// 🔵 Entrada: 1
// 🔵 Entrada: 2
// 🔵 Entrada: 3
// 🔵 Entrada: 4
// 🔵 Entrada: 5
// ✅ Saída: Sem valor
```

## Cenário 2: Valor diferente é emitido do que o esperado

- **Sintoma**: Valor diferente do esperado é emitido.

### Causa 1: Operador está na ordem errada.

O resultado depende da ordem em que os operadores são aplicados.

```ts
import { of } from 'rxjs';
import { map, filter } from 'rxjs';

// ❌ Resultado diferente do esperado
of(1, 2, 3, 4, 5)
  .pipe(
    map(x => x * 2),     // 2, 4, 6, 8, 10
    filter(x => x < 5)   // Apenas 2, 4 passam
  )
  .subscribe(value => console.log('Resultado:', value));
// Saída: 2, 4

// ✅ Ordem correta
of(1, 2, 3, 4, 5)
  .pipe(
    filter(x => x < 5),  // Apenas 1, 2, 3, 4 passam
    map(x => x * 2)      // 2, 4, 6, 8
  )
  .subscribe(value => console.log('Resultado:', value));
// Saída: 2, 4, 6, 8
```

### Causa 2: Mudanças não intencionais devido a referências compartilhadas

Porque objetos JavaScript são passados por referência, é possível modificar o objeto original.

```ts
import { of } from 'rxjs';
import { map } from 'rxjs';

interface User {
  id: number;
  name: string;
}

const user: User = { id: 1, name: 'Alice' };

of(user)
  .pipe(
    // ❌ Modifica o objeto original diretamente
    map(u => {
      u.name = 'Bob'; // Objeto original é modificado
      return u;
    })
  )
  .subscribe(value => console.log('Após mudança:', value));

console.log('Objeto original:', user); // { id: 1, name: 'Bob' }

// ✅ Criar um novo objeto
of(user)
  .pipe(
    map(u => ({ ...u, name: 'Charlie' })) // Novo objeto com sintaxe spread
  )
  .subscribe(value => console.log('Após mudança:', value));

console.log('Objeto original:', user); // { id: 1, name: 'Alice' } (não modificado)
```

### Causa 3: Timing de processamento assíncrono

A ordem de conclusão do processamento assíncrono pode ser diferente do esperado.

```ts
import { of, delay } from 'rxjs';
import { mergeMap, tap } from 'rxjs';

// ❌ Não espera a conclusão do processamento assíncrono
of(1, 2, 3)
  .pipe(
    tap(value => console.log('Início:', value)),
    mergeMap(value =>
      of(value * 2).pipe(
        delay(100 - value * 10) // Valores maiores completam mais rápido
      )
    )
  )
  .subscribe(value => console.log('Completo:', value));

// Saída:
// Início: 1
// Início: 2
// Início: 3
// Completo: 3  ← Delay mais curto
// Completo: 2
// Completo: 1  ← Delay mais longo

// ✅ Garantir ordem
import { concatMap } from 'rxjs';

of(1, 2, 3)
  .pipe(
    tap(value => console.log('Início:', value)),
    concatMap(value =>  // mergeMap → concatMap
      of(value * 2).pipe(delay(100 - value * 10))
    )
  )
  .subscribe(value => console.log('Completo:', value));

// Saída:
// Início: 1
// Completo: 1
// Início: 2
// Completo: 2
// Início: 3
// Completo: 3
```

## Cenário 3: Subscription não completada (stream infinito)

- **Sintoma**: `complete` não é chamado e o stream não é terminado

Você precisa completá-lo explicitamente, já que `interval`, `fromEvent`, etc. continuam emitindo valores indefinidamente.

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

// ❌ interval continua a emitir valores indefinidamente
interval(1000)
  .pipe(
    tap(value => console.log('Valor:', value))
  )
  .subscribe({
    complete: () => console.log('Esta linha não é executada')
  });

// ✅ Completar explicitamente com take
import { take } from 'rxjs';

interval(1000)
  .pipe(
    take(5), // Completar após 5 valores
    tap(value => console.log('Valor:', value))
  )
  .subscribe({
    complete: () => console.log('Complete')
  });
```

### Técnicas de Depuração

Defina um timeout para parar o stream infinito ao depurar.

```ts
import { interval, timer } from 'rxjs';
import { tap, takeUntil } from 'rxjs';

// Definir timeout para depuração
const stop$ = timer(5000); // Completar após 5 segundos

interval(1000)
  .pipe(
    takeUntil(stop$),
    tap({
      next: value => console.log('Valor:', value),
      complete: () => console.log('Parado no timeout')
    })
  )
  .subscribe();
```

## Cenário 4: Vazamento de memória (esqueceu de fazer unsubscribe)

- **Sintoma**: Aplicação fica gradualmente mais lenta

### Causa: Subscriptions não canceladas que não são mais necessárias

Um vazamento de memória ocorre quando uma subscription permanece após um componente ou serviço ser destruído.

```ts
import { interval } from 'rxjs';

class UserComponent {
  private subscription: any;

  ngOnInit() {
    // ❌ Esqueceu de fazer unsubscribe
    interval(1000).subscribe(value => {
      console.log('Valor:', value); // Continua a executar após o componente ser destruído
    });
  }

  ngOnDestroy() {
    // Sem unsubscription
  }
}

// ✅ Gerenciar subscriptions adequadamente
class UserComponentFixed {
  private subscription: any;

  ngOnInit() {
    this.subscription = interval(1000).subscribe(value => {
      console.log('Valor:', value);
    });
  }

  ngOnDestroy() {
    // Unsubscribe quando o componente é destruído
    if (this.subscription) {
      this.subscription.unsubscribe();
    }
  }
}
```

**Padrão recomendado: use `takeUntil`**.

O padrão `takeUntil` pode ser usado para automatizar unsubscriptions.

```ts
import { interval, Subject } from 'rxjs';
import { takeUntil } from 'rxjs';

class UserComponentBest {
  private destroy$ = new Subject<void>();

  ngOnInit() {
    // ✅ Automaticamente fazer unsubscribe com takeUntil
    interval(1000)
      .pipe(
        takeUntil(this.destroy$)
      )
      .subscribe(value => console.log('Valor:', value));
  }

  ngOnDestroy() {
    this.destroy$.next();
    this.destroy$.complete();
  }
}
```

### Detecção de vazamento de memória

Rastreie o número de subscriptions com um operador personalizado.

```ts
import { interval } from 'rxjs';
import { tap } from 'rxjs';

let subscriptionCount = 0;

const trackSubscriptions = <T>() =>
  tap<T>({
    subscribe: () => {
      subscriptionCount++;
      console.log('📈 Subscriptions:', subscriptionCount);
    },
    unsubscribe: () => {
      subscriptionCount--;
      console.log('📉 Subscriptions:', subscriptionCount);
    }
  });

// Exemplo de uso
const stream$ = interval(1000).pipe(
  trackSubscriptions()
);

const sub1 = stream$.subscribe();
// Saída: 📈 Subscriptions: 1

const sub2 = stream$.subscribe();
// Saída: 📈 Subscriptions: 2

setTimeout(() => {
  sub1.unsubscribe();
  // Saída: 📉 Subscriptions: 1
}, 3000);
```

## Cenário 5: Você não nota um erro

- **Sintoma**: Erro ocorre, mas não é exibido e é ignorado

Sem um error handler, o erro pode ser suprimido e não notado.

```ts
import { of, throwError } from 'rxjs';
import { mergeMap, catchError } from 'rxjs';

// ❌ Erro é suprimido porque não há tratamento de erro
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Erro'));
      }
      return of(value);
    })
  )
  .subscribe(); // Sem error handler

// ✅ Tratamento de erro adequado
of(1, 2, 3)
  .pipe(
    mergeMap(value => {
      if (value === 2) {
        return throwError(() => new Error('Erro'));
      }
      return of(value);
    }),
    catchError((error: unknown) => {
      console.error('🔴 Erro capturado:', (error instanceof Error ? error.message : String(error)));
      return of(-1); // Valor de fallback
    })
  )
  .subscribe({
    next: value => console.log('Valor:', value),
    error: error => console.error('🔴 Erro no subscribe:', error)
  });

// Saída:
// Valor: 1
// 🔴 Erro capturado: Erro
// Valor: -1
```

### Configurar error handler global

Um handler global pode ser configurado para capturar todos os erros pendentes.

```ts
import { Observable } from 'rxjs';

// Capturar todos os erros não tratados
const originalCreate = Observable.create;

Observable.create = function(subscribe: any) {
  return originalCreate.call(this, (observer: any) => {
    try {
      return subscribe(observer);
    } catch (error) {
      console.error('🔴 Erro não tratado:', error);
      observer.error(error);
    }
  });
};
```

## Cenário 6: Desejo rastrear tentativas de retry

- **Sintoma**: Estou usando o operador `retry`, mas não sei quantas tentativas de retry estou obtendo.

Ao tentar novamente automaticamente quando um erro ocorre, rastrear quantas tentativas de retry são realmente executadas facilitaria a depuração e o logging.

### Depuração Básica de Retry

Use `retryWhen` para registrar o número de tentativas de retry.

```ts
import { throwError, of, timer } from 'rxjs';
import { retryWhen, mergeMap, tap } from 'rxjs';

throwError(() => new Error('Erro temporário'))
  .pipe(
    retryWhen((errors) =>
      errors.pipe(
        mergeMap((error, index) => {
          const retryCount = index + 1;
          console.log(`🔄 Tentativa de retry ${retryCount}`);

          if (retryCount > 2) {
            console.log('❌ Contagem máxima de retry atingida');
            throw error;
          }

          return timer(1000);
        })
      )
    )
  )
  .subscribe({
    next: value => console.log('✅ Sucesso:', value),
    error: error => console.log('🔴 Erro final:', error.message)
  });

// Saída:
// 🔄 Tentativa de retry 1
// 🔄 Tentativa de retry 2
// 🔄 Tentativa de retry 3
// ❌ Contagem máxima de retry atingida
// 🔴 Erro final: Erro temporário
```

> [!TIP]
> Para padrões de implementação mais detalhados sobre depuração de retries, consulte a seção "Depurando Retries" de [retry e catchError](/pt/guide/error-handling/retry-catch#debugging-retries).
> - Rastreamento básico usando o callback de erro tap
> - Logging detalhado com retryWhen
> - Exponential backoff e logging
> - Objeto de configuração retry do RxJS 7.4+

## Resumo

Soluções para cenários comuns de depuração:

- ✅ **valores não fluem** → esqueceu de fazer subscribe, verifique condições de filtragem
- ✅ **Valor diferente do esperado** → cuidado com ordem do operador, compartilhamento de referência
- ✅ **Subscription não completada** → use `take` ou `takeUntil` para streams infinitos
- ✅ **Vazamento de memória** → auto unsubscribe com padrão `takeUntil`
- ✅ **Erros perdidos** → implemente tratamento de erro adequado
- ✅ **rastreamento de retry** → logging com `retryWhen` ou objeto de configuração

## Páginas Relacionadas

- [Estratégias Básicas de Depuração](/pt/guide/debugging/) - Como usar operador tap e ferramentas do desenvolvedor
- [Ferramentas de Debug Personalizadas](/pt/guide/debugging/custom-tools) - Streams nomeados, operadores de debug
- [Depuração de Desempenho](/pt/guide/debugging/performance) - Monitoramento de subscription, verificação de uso de memória
- [Tratamento de Erros](/pt/guide/error-handling/strategies) - Estratégias de tratamento de erros
