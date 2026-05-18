---
description: "combineLatestAll é um operador que usa um Observable de ordem superior (Observable of Observable) e combina o valor mais recente de cada um deles quando todos os Observable internos tiverem sido acionados pelo menos uma vez."
---

# combineLatestAll - combina os valores mais recentes do Observable interno

O operador `combineLatestAll` usa um **Higher-order Observable** (Observável de Observáveis),
**Uma vez que todos os Observable internos tenham sido acionados pelo menos uma vez**, os **Latest Values** de cada um são combinados e gerados como uma **Array**.

## Sintaxe básica e uso

```ts
import { interval, of } from 'rxjs';
import { combineLatestAll, take } from 'rxjs';

// 3Dois internosObservablecomHigher-order Observable
const higherOrder$ = of(
  interval(1000).pipe(take(3)), // 0, 1, 2
  interval(500).pipe(take(4)),  // 0, 1, 2, 3
  interval(2000).pipe(take(2))  // 0, 1
);

// Todos os internosObservabletêm pelo menos1dispara uma vez, combine os valores mais recentes
higherOrder$
  .pipe(combineLatestAll())
  .subscribe(values => console.log(values));

// Saída:
// [1, 3, 0] ← Quando todos tiverem disparado pelo menos1Quando todos tiverem disparado pelo menos uma vez (após2segundos depois)
// [2, 3, 0] ← 1Quando o segundoObservableé disparado (após 2 segundos), o segundo2dispara (após3segundos depois)
// [2, 3, 1] ← 3Quando o segundoObservableé disparado (após 2 segundos), o segundo1dispara (após4segundos depois)
```

- Coletar Observables internos quando o Observable de ordem superior estiver **completo**.
- Comece a combinar quando todos os Observable internos tiverem disparado** pelo menos uma vez
- Sempre que qualquer Observable interno emitir um valor, **combine** todos os últimos valores e **output**.

[🌐 Documentação oficial do RxJS - `combineLatestAll`](https://rxjs.dev/api/index/function/combineLatestAll)

## 💡 Padrão de utilização típico.

- **Combinar os resultados mais recentes de várias chamadas de API**
- **Sincronizar os valores mais recentes de várias entradas de formulário**
- Integrar várias fontes de dados em tempo real**

## Exemplos práticos de código

Exemplo de combinação dos resultados mais recentes de várias chamadas de API

```ts
import { of, timer, Observable } from 'rxjs';
import { map, combineLatestAll, take } from 'rxjs';

const output = document.createElement('div');
document.body.appendChild(output);

// 3Dois simuladosAPIChamada de criação (Higher-order Observable)
const apiCalls$: Observable<Observable<string>> = of(
  // API 1: Informações do usuário (1Concluído em segundos,3atualizado uma vez)
  timer(0, 1000).pipe(
    take(3),
    map(n => `Usuário: User${n}`)
  ),
  // API 2: Número de notificações (0.5Concluído em segundos,4atualizado uma vez)
  timer(0, 500).pipe(
    take(4),
    map(n => `Notificações: ${n}Número de notificações`)
  ),
  // API 3: Status2Concluído em segundos,2atualizado uma vez)
  timer(0, 2000).pipe(
    take(2),
    map(n => n === 0 ? 'Status: Off-line' : 'Status: Online')
  )
);

// Todas as chamadasAPIValor mais recente combinado de chamadas
apiCalls$
  .pipe(combineLatestAll())
  .subscribe(values => {
    output.innerHTML = '<strong>Status mais recente:</strong><br>';
    values.forEach((value, index) => {
      const item = document.createElement('div');
      item.textContent = `${index + 1}. ${value}`;
      output.appendChild(item);
    });
  });
```

- Três chamadas de API são executadas em paralelo.
- Depois de todas terem sido disparadas pelo menos uma vez**, os resultados combinados são exibidos
- Sempre que qualquer API for atualizada, a **última combinação** será exibida

## Creation Function relacionada

No entanto, a função `combineLatestAll` é usada principalmente para achatar Observable de ordem superior,
Use a Creation Function `combineLatest` para combinações normais de Observables de várias ordens.

```ts
import { combineLatest, interval } from 'rxjs';

// Creation FunctionEdição (uso mais geral)
const combined$ = combineLatest([
  interval(1000),
  interval(500),
  interval(2000)
]);

combined$.subscribe(console.log);
```

Consulte [Capítulo 3 Creation Function - combineLatest](/pt/guide/creation-functions/combination/combineLatest).

## 🔄 Operadores relacionados.

| Operador. | Descrição. |
|---|---|
| [mergeAll](. /mergeAll) | Assina todos os Observable internos em paralelo. |
| [concatAll](. /concatAll) | Assinar os Observable internos em sequência. |
| [switchAll](. /switchAll) | Mudar para um novo Observable interno. |
| [zipAll](. /zipAll) | Emparelhar os valores de cada Observable interno na ordem correspondente |

## ⚠️ Notas.

### O Observable de ordem superior deve ser preenchido.

O `combineLatestAll` aguarda a coleção de Observables internos **até** que o Observable de ordem superior (Observable externo) seja **completado**.

#### ❌ O Observable de ordem superior não é concluído, portanto, nada é gerado.

```ts
interval(1000).pipe(
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log); // Nada é emitido
```

#### ✅ Take to complete

```ts
interval(1000).pipe(
  take(3), // 3Concluído em uma sessão
  map(() => of(1, 2, 3)),
  combineLatestAll()
).subscribe(console.log);
```

### Todos os Observable internos devem disparar pelo menos uma vez

Nenhum valor é gerado até que todos os Observable internos tenham sido acionados **pelo menos uma vez**.

```ts
// 1Se qualquer um dos status internosObservableNada será emitido se houver
of(
  of(1, 2, 3),
  NEVER // Nunca dispara para sempre.
).pipe(
  combineLatestAll()
).subscribe(console.log); // Nada é emitido
```

### Uso de memória.

Observe o uso da memória se houver muitos Observable internos, pois os **valores mais recentes de todos os Observable internos são mantidos na memória**.
