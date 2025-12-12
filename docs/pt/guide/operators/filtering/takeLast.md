---
description: takeLast é um operador de filtragem do RxJS que emite apenas os últimos N valores quando o stream Observable é completado. É ideal para cenários onde apenas os últimos valores de todo o stream são necessários, como obter as entradas de log mais recentes, exibir os N principais itens em um placar e resumos de dados finais na conclusão. Não pode ser usado com streams infinitos porque mantém valores em um buffer até a conclusão.
---

# takeLast - Obter os Últimos N Valores

O operador `takeLast` emite apenas os últimos N valores quando o stream **completa**. Ele mantém valores em um buffer até que o stream complete e então os emite todos de uma vez.


## 🔰 Sintaxe Básica e Uso

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9
```

**Fluxo de operação**:
1. Stream emite 0, 1, 2, 3, 4, 5, 6, 7, 8, 9
2. Internamente mantém os últimos 3 valores no buffer
3. Stream completa
4. Emite valores do buffer 7, 8, 9 em ordem

[🌐 Documentação Oficial do RxJS - `takeLast`](https://rxjs.dev/api/operators/takeLast)


## 🆚 Contraste com take

`take` e `takeLast` têm comportamento contrastante.

```ts
import { range } from 'rxjs';
import { take, takeLast } from 'rxjs';

const numbers$ = range(0, 10); // 0 a 9

// take: Obter os primeiros N valores
numbers$.pipe(
  take(3)
).subscribe(console.log);
// Saída: 0, 1, 2 (saída imediata)

// takeLast: Obter os últimos N valores
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9 (saída após aguardar conclusão)
```

| Operador | Posição de Obtenção | Momento da Saída | Comportamento Antes da Conclusão |
|---|---|---|---|
| `take(n)` | Primeiros n valores | Saída imediata | Completa automaticamente após n valores |
| `takeLast(n)` | Últimos n valores | Saída toda junta após conclusão | Mantém no buffer |


## 💡 Padrões de Uso Típicos

1. **Obter as Últimas N Entradas de Log**
   ```ts
   import { from } from 'rxjs';
   import { takeLast } from 'rxjs';

   interface LogEntry {
     timestamp: number;
     level: 'info' | 'warn' | 'error';
     message: string;
   }

   const logs$ = from([
     { timestamp: 1, level: 'info' as const, message: 'Aplicativo iniciado' },
     { timestamp: 2, level: 'info' as const, message: 'Usuário fez login' },
     { timestamp: 3, level: 'warn' as const, message: 'Consulta lenta detectada' },
     { timestamp: 4, level: 'error' as const, message: 'Falha na conexão' },
     { timestamp: 5, level: 'info' as const, message: 'Retentativa bem-sucedida' },
   ] as LogEntry[]);

   // Obter as últimas 3 entradas de log
   logs$.pipe(
     takeLast(3)
   ).subscribe(log => {
     console.log(`[${log.level}] ${log.message}`);
   });
   // Saída:
   // [warn] Consulta lenta detectada
   // [error] Falha na conexão
   // [info] Retentativa bem-sucedida
   ```

2. **Obter os N Principais no Placar**
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
   ] as Score[]);

   // Obter os 3 principais
   scores$.pipe(
     takeLast(3)
   ).subscribe(score => {
     console.log(`${score.player}: ${score.score}`);
   });
   // Saída: Charlie: 200, Dave: 180, Eve: 220
   ```


## ⚠️ Observações Importantes

> [!WARNING]
> `takeLast` **aguarda até que o stream complete**, portanto não funciona com streams infinitos. Além disso, se n em `takeLast(n)` for grande, consome muita memória.

### 1. Não Pode Usar com Streams Infinitos

`takeLast` aguarda a conclusão do stream, portanto não funciona com streams infinitos.

```ts
import { interval } from 'rxjs';
import { takeLast } from 'rxjs';

// ❌ Exemplo ruim: Usar takeLast com stream infinito
interval(1000).pipe(
  takeLast(3)
).subscribe(console.log);
// Nada emitido (porque o stream nunca completa)
```

**Solução**: Torne-o um stream finito combinando com `take`

```ts
import { interval } from 'rxjs';
import { take, takeLast } from 'rxjs';

// ✅ Bom exemplo: Usar takeLast após torná-lo stream finito
interval(1000).pipe(
  take(10),      // Completar com os primeiros 10 valores
  takeLast(3)    // Obter os últimos 3 deles
).subscribe(console.log);
// Saída: 7, 8, 9
```

### 2. Seja Consciente do Uso de Memória

`takeLast(n)` mantém os últimos n valores em um buffer, portanto n grande consome memória.

```ts
import { range } from 'rxjs';
import { takeLast } from 'rxjs';

// ⚠️ Cuidado: Manter grande quantidade de dados no buffer
range(0, 1000000).pipe(
  takeLast(100000) // Manter 100.000 itens na memória
).subscribe(console.log);
```


## 🎯 Diferença em relação ao last

```ts
import { range } from 'rxjs';
import { last, takeLast } from 'rxjs';

const numbers$ = range(0, 10);

// last: Apenas o último 1 valor
numbers$.pipe(
  last()
).subscribe(console.log);
// Saída: 9

// takeLast(1): Último 1 valor (saída como valor único, não array)
numbers$.pipe(
  takeLast(1)
).subscribe(console.log);
// Saída: 9

// takeLast(3): Últimos 3 valores
numbers$.pipe(
  takeLast(3)
).subscribe(console.log);
// Saída: 7, 8, 9
```

| Operador | Contagem de Obtenção | Especificação de Condição | Caso de Uso |
|---|---|---|---|
| `last()` | 1 valor | Possível | Último 1 valor ou último 1 valor que atende à condição |
| `takeLast(n)` | n valores | Não possível | Simplesmente obter os últimos n valores |


## 🎓 Resumo

### Quando Usar takeLast
- ✅ Quando você precisa dos últimos N dados do stream
- ✅ Quando você quer obter as últimas N entradas de logs ou transações
- ✅ Quando a conclusão do stream é garantida
- ✅ Quando você quer exibir resumo de dados ou os N principais itens

### Quando Usar take
- ✅ Quando você precisa dos primeiros N dados do stream
- ✅ Quando você quer obter resultados imediatamente
- ✅ Quando você quer obter uma parte de um stream infinito

### Observações
- ⚠️ Não pode usar com streams infinitos (não completa)
- ⚠️ N grande em `takeLast(n)` consome memória
- ⚠️ A saída é feita toda junta após a conclusão (não emite imediatamente)
- ⚠️ Frequentemente precisa combinar com `take(n)` para tornar o stream finito


## 🚀 Próximos Passos

- **[take](/pt/guide/operators/filtering/take)** - Aprenda como obter os primeiros N valores
- **[last](/pt/guide/operators/filtering/last)** - Aprenda como obter o último 1 valor
- **[skip](/pt/guide/operators/filtering/skip)** - Aprenda como pular os primeiros N valores
- **[filter](/pt/guide/operators/filtering/filter)** - Aprenda como filtrar com base em condições
- **[Exemplos Práticos de Operadores de Filtragem](/pt/guide/operators/filtering/practical-use-cases)** - Aprenda casos de uso reais
