---
description: "takeLast é um operador de filtragem RxJS que gera apenas os últimos N valores quando um fluxo Observable é concluído. É ideal para situações em que apenas o último valor de todo o fluxo é necessário, como obter a última contagem no registro, exibir os N valores principais na tabela de classificação ou o resumo final dos dados na conclusão. Não pode ser usado com fluxos infinitos, pois é mantido em um buffer até a conclusão."
---

# TakeLast - obtém os últimos N valores

O operador `takeLast` gera apenas os últimos N valores no momento em que o fluxo é **completado**. Ele mantém os valores em um buffer até que o fluxo seja concluído e os envia juntos após a conclusão.

## Sintaxe básica e uso

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (para)9para

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9
```

**Fluxo de operação**:.
1. o fluxo emite 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. internamente mantém os últimos 3 no buffer
3. fluxo concluído 4. valores do buffer 7, 8, 9
4. saída dos valores de buffer 7, 8, 9 em sequência

[🌐 Documentação oficial do RxJS - `takeLast`](https://rxjs.dev/api/operators/takeLast)

## 🆚 Contraste com take.

O `take` e o `takeLast` têm comportamentos diferentes.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (para)9para

// take: O primeiroNObter o primeiro
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Saída: 0, 1, 2(saída imediata)

// takeLast: Obter o últimoNObter o primeiro
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9(aguardar a conclusão antes de gerar a saída)
```

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0de (para)9para

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9

## 💡 Padrão de utilização típico

1. **Obter as N entradas de registro mais recentes**.

```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'App started' },
     { timestamp: 2, level: 'info' as const, message: 'User logged in' },
     { timestamp: 3, level: 'warn' as const, message: 'Slow query detected' },
     { timestamp: 4, level: 'error' as const, message: 'Connection failed' },
     { timestamp: 5, level: 'info' as const, message: 'Retry successful' },
   ] as LogEntry[]);

   // Obter o mais recente3Obter os registros mais recentes
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Saída:
   // [warn] Slow query detected
   // [error] Connection failed
   // [info] Retry successful
   ```

2. **Topo da tabela de classificaçãoNRecuperar o topo**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface Score {
     player: string;
     score: number;
   }

   const scores$ = from([
     { player: 'Alice', score: 100 },
     { player: 'Bob', score: 150 },
     { player: 'Charlie', score: 200 },
     { player: 'Dave', score: 180 },
     { player: 'Eve', score: 220 }
   ] as Score[]).pipe(
     // Suponha que esteja classificado por pontuação
   );

   // Obter o topo3Recuperar o
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Saída: Charlie: 200, Dave: 180, Eve: 220
   ```

3. **Resumo final após a conclusão do processamento de dadosNResumo dos casos**
   ```ts
   import { interval } from 'rxjs';
   import { take, map, takeLast } from 'rxjs';

   // Simulação de dados do sensor
   const sensorData$ = interval(100).pipe(
     take(20),
     map(i => ({
       id: i,
       temperature: 20 + Math.random() * 10
     }))
   );

   // Obter o último5Cálculo da temperatura média do caso
   sensorData$.pipe(
     takeLast(5)
   ).subscribe({
     next: data => {
       console.log(`Dados${data.id}: ${data.temperature.toFixed(1)}°C`);
     },
     complete: () => {
       console.log('Último5Aquisição de dados do caso concluída');
     }
   });
   ```

## 🧠 Exemplo prático de código (histórico de entrada)

Exemplo de exibição do mais recente3Este é um exemplo de exibição dos valores mais recentes inseridos pelo usuário.

```

ts.
import { fromEvent, Subject } from 'rxjs';
import { takeLast } from 'rxjs';

// Criação de elementos da interface do usuário
const container = document.createElement('div');
document.body.appendChild(container);

const input = document.createElement('input');
input.placeholder = 'Insira um valor e Enter';
container.appendChild(input);

const submitButton = document.createElement('button');
submitButton.textContent = 'Mostrar histórico (últimos 3)';
contêiner.appendChild(submitButton);

const historyDisplay = document.createElement('div');
historyDisplay.style.marginTop = '10px';
contêiner.appendChild(historyDisplay);

// Subject para armazenar valores de entrada
const inputs$ = new Subject();.

// **IMPORTANTE**: defina primeiro a assinatura takeLast
inputs$.pipe(
  takeLast(3)
).subscribe({
  next: (value) => {
    const item = document.createElement('div');
    item.textContent = `- ${value}`;
    historyDisplay.appendChild(item);
  },.
  complete: () => {
    const note = document.createElement('div');
    note.style.marginTop = '5px';
    note.style.colour = 'grey';
    note.textContent = '(Recarregue a página para digitar novamente)';
    historyDisplay.appendChild(note);

    // Desativar campos de entrada e botões
    input.disabled = true;
    submitButton.disabled = true;
  }
});

// Adicionar entrada com a tecla Enter
fromEvent<KeyboardEvent>(input, 'keydown').subscribe(event => {
  if (event.key === 'Enter' && input.value.trim()) {
    inputs$.next(input.value);
    console.log(`Adicionar: ${input.value}`);
    input.value = '';
  }
});

// Completar com o clique do botão e exibir o histórico
fromEvent(submitButton, 'click').subscribe(() => {
  historyDisplay.innerHTML = '<strong>Histórico (últimos 3):</strong><br>';
  inputs$.complete(); // fluxo completo → takeLast dispara
});

```

> [!IMPORTANT]
> **Pontos principais**:
> - `takeLast(3)` Assine o**primeiro.**deve ser configurado primeiro
> - quando o botão for clicado. `complete()` o último dos valores recebidos até aquele momento será emitido.3O último valor recebido até aquele momento é emitido.
> - `complete()` Após a chamada**Depois de chamar**para `subscribe` os valores não fluem.

## ⚠️ Um ponto importante a ser observado

> [!WARNING]
> `takeLast` é esperar até que o fluxo**Esperar até a conclusão**Portanto, ele não funciona com fluxos infinitos. Além disso, o`takeLast(n)` doné grande e consome muita memória.

### 1. Não pode ser usado com fluxos infinitos.

`takeLast` não funciona com fluxos infinitos porque espera até que o fluxo seja concluído.

```

ts.
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Exemplo ruim: usando takeLast com fluxos infinitos
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);.
// Nada é gerado (porque o fluxo nunca é concluído)

```

**Solução.**: `take` Use um fluxo finito em combinação com

```

ts.
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Bom exemplo: fluxo finito e, em seguida, use takeLast
interval(1000).pipe(
  take(10), // Complete com os primeiros 10
  takeLast(3) // pegar os últimos 3
).subscribe(console.log);.
// Saída: 7, 8, 9

```

### 2. Preste atenção ao uso da memória

`takeLast(n)` não funciona com fluxos finitos porque mantém a últimanA última peça a ser mantida no buffer,nSe o buffer for grande, ele consome mais memória.

```

ts.
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Observação: grandes quantidades de dados são mantidas em um buffer
range(0, 1000000).pipe(
  takeLast(100000) // 100.000 registros mantidos na memória
).subscribe(console.log);.

```

## 🎯 last A diferença entre o

```

ts.
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: apenas o último
numbers$.pipe(
  last()
).subscribe(console.log);
// saída: 9

// takeLast(1): último (saída como valor único, não como matriz)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);.
// Saída: 9

// takeLast(3): último 3
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9

```

| operador | Número de aquisições | Especificação da condição | Caso de uso |
|---|---|---|---|
| `last()` | 1Número de | Possíveis | Obter o último1Peças ou a última peça que atende à condição1Número de |
| `takeLast(n)` | nNúmero de | Impossível | Obter o últimonBasta obter a última peça que satisfaça a condição |

## 📋 Uso com segurança de tipo

TypeScript Este é um exemplo de uma implementação segura para o tipo que faz uso de genéricos em

```

ts.
import { Observable, from } from 'rxjs';
import { takeLast } from 'rxjs';

interface Transaction {
  id: string;
  amount: número;
  timestamp: data;
  status: 'pending' | 'completed' | 'failed'; }
}

function getRecentTransactions(
  transactions$: Observable,.
  count: number
): Observable {
  return transactions$.pipe(
    takeLast(count)
  );
}

// Exemplo de uso
const transactions$ = from([.
  { id: '1', amount: 100, timestamp: new Date('2025-01-01'), status: 'completed' as const }
  { id: '2', amount: 200, timestamp: new Date('2025-01-02'), status: 'completed' as const }
  { id: '3', amount: 150, timestamp: new Date('2025-01-03'), status: 'pending' as const }
  { id: '4', amount: 300, timestamp: new Date('2025-01-04'), status: 'completed' as const }
  { id: '5', amount: 250, timestamp: new Date('2025-01-05'), status: 'failed' as const }
] as Transaction[]);.

// Obter as três transações mais recentes
getRecentTransactions(transactions$, 3).subscribe(tx => {
  console.log(`${tx.id}: ${tx.amount} yen (${tx.status})`);
});
// Saída:.
// 3: 150 ienes (pendente)
// 4: 300 ienes (concluído)
// 5: ¥250 (falhou)

```

## 🔄 skip e takeLast combinação de

A parte do meio do valor é excluída e somente a última parte pode ser recuperada.NSomente o último valor pode ser recuperado.

```

ts
import { range } from 'rxjs';
import { skip, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

// pular os primeiros 5 e pegar os últimos 3 restantes
numbers$.pipe(
  skip(5), // skip 0, 1, 2, 3, 4
  takeLast(3) // pega os últimos 3 dos 5, 6, 7, 8, 9 restantes
).subscribe(console.log);.
// Saída: 7, 8, 9
```

## 🎓 Resumo

### Quando takeLast deve ser usado.
- Se você precisar dos últimos N dados em um fluxo
- Se você quiser obter os últimos N registros ou transações
- Se houver garantia de que o fluxo será concluído
- Se quiser exibir um resumo ou os N registros principais de dados

### Quando você deve usar o take.
- Se você precisar dos primeiros N dados no fluxo
- Se você quiser obter os resultados imediatamente
- Se você quiser obter parte de um fluxo infinito

### Notas.
- ⚠️ Não pode ser usado com fluxos infinitos (pois eles não se completam)
- ⚠️ Um n grande em `takeLast(n)` consome memória
- ⚠️ A saída é compilada após a conclusão (não imediatamente)
- ⚠️ Frequentemente precisa ser combinado com o `take(n)` para criar um fluxo finito

## Próxima etapa.

- **[take](. /take)** - saiba como obter os primeiros n valores.
- **[last](. /last)** - saiba como obter o último 1 valor
- **[skip](. /skip)** - saiba como pular os primeiros N valores
- **[filter](. /filter)** - aprenda a filtrar com base em condições
- **[filtering-operator-practical-use-cases](. /practical-use-cases)** - aprenda a usar casos de uso reais
